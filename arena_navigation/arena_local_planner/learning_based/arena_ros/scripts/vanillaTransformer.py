from torch.nn.modules import TransformerEncoderLayer, TransformerEncoder, TransformerDecoder, TransformerDecoderLayer
from torch.nn import MultiheadAttention
import math
import torch
from torch import nn
from torch.nn.modules.normalization import LayerNorm
from torch.nn.init import xavier_uniform_
from torch.nn.modules import Linear
from torch.autograd import Variable
import torch.nn.functional as F
from torch.nn import Dropout

'''
GRU gating layer used in Stabilizing transformers in RL.
Note that all variable names follow the notation from section: "Gated-Recurrent-Unit-type gating" 
in https://arxiv.org/pdf/1910.06764.pdf
'''
class GRUGate(nn.Module):

    def __init__(self,d_model):
        #d_model is dimension of embedding for each token as input to layer (want to maintain this in the gate)
        super(GRUGate,self).__init__()

        self.linear_w_r = Linear(d_model,d_model,bias=False)
        self.linear_u_r = Linear(d_model,d_model,bias=False)
        self.linear_w_z = Linear(d_model,d_model)               ### Giving bias to this layer (will count as b_g so can just initialize negative)
        self.linear_u_z = Linear(d_model, d_model,bias=False)
        self.linear_w_g = Linear(d_model, d_model,bias=False)
        self.linear_u_g = Linear(d_model, d_model,bias=False)

    def forward(self,x,y):
        ### Here x,y follow from notation in paper

        z = torch.sigmoid(self.linear_w_z(y) + self.linear_u_z(x))  #MAKE SURE THIS IS APPLIED ON PROPER AXIS
        r = torch.sigmoid(self.linear_w_r(y) + self.linear_u_r(x))
        h_hat = torch.tanh(self.linear_w_g(y) + self.linear_u_g(r*x))  #Note elementwise multiplication of r and x
        return (1.-z)*x + z*h_hat


class StableTransformerLayer(nn.Module):
    def __init__(self, d_model, nhead, dim_feedforward=2048, dropout=0.1, use_gate = False):
        #fill in reordering of operations as done in https://arxiv.org/pdf/1910.06764.pdf
        #d_model: dimension of embedding for each input
        super(StableTransformerLayer,self).__init__()

        self.use_gate = use_gate
        self.gate_mha = GRUGate(d_model)
        self.gate_mlp = GRUGate(d_model)
        self.self_attn = MultiheadAttention(d_model, nhead, dropout=dropout)

        self.linear1 = Linear(d_model, dim_feedforward)
        self. dropout = Dropout(dropout)
        self.linear2 = Linear(dim_feedforward, d_model)

        self.norm1 = LayerNorm(d_model)
        self.norm2 = LayerNorm(d_model)
        self.dropout1 = Dropout(dropout)
        self.dropout2 = Dropout(dropout)

        self.activation = F.relu

    def forward(self, src, src_mask=None, src_key_padding_mask=None):

        '''
        #ORIGINAL TRANSFORMER ORDERING
        src2 = self.self_attn(src, src, src, attn_mask=src_mask,
                              key_padding_mask=src_key_padding_mask)[0]
        src = src + self.dropout1(src2)
        src = self.norm1(src)
        src2 = self.linear2(self.dropout(self.activation(self.linear1(src))))
        src = src + self.dropout2(src2)
        src = self.norm2(src)
        '''

        #HOW SHOULD THE DROPOUT BE APPLIED, it seems like dropout is completely missing the original source that residually connects?
        #This doesn't perfectly correspond to dropout used in TransformerXL I believe. (to do: read their code)


        src2 = self.norm1(src)
        src2 = self.self_attn(src2, src2, src2, attn_mask=src_mask,
                              key_padding_mask=src_key_padding_mask)[0]
        if self.use_gate:
            src2 = self.gate_mha(src, F.relu(self.dropout1(src2)))
        else:
            src2 = src + F.relu(self.dropout1(src2))

        src3 = self.norm2(src2)
        src3 = self.linear2(self.dropout(self.activation(self.linear1(src3))))

        if self.use_gate:
            src3 = self.gate_mlp(src2, self.dropout2(F.relu(src3)))
        else:
            src3 = self.dropout2(F.relu(src3)) + src2

        return src3



# Slightly extend the transformer model to embed encoder and decoder initial inputs
# and use positional encodings as well
# some code here copied from https://pytorch.org/docs/master/_modules/torch/nn/modules/transformer.html#Transformer
class StableTransformerEncoder(nn.Module):

    def __init__(self, d_model, nhead, num_encoder_layers, num_decoder_layers,
                 dim_feedforward, dropout, activation, embedding_layer):
        super(TransformerModelEncoder, self).__init__()
        self.pos_encoder = PositionalEncoding(d_model=d_model, dropout=0.1)  # , max_len=100)
        encoder_layer = TransformerEncoderLayer(d_model, nhead, dim_feedforward, dropout, activation)
        encoder_norm = LayerNorm(d_model)
        self.encoder = TransformerEncoder(encoder_layer, num_encoder_layers, encoder_norm)

        self.d_model = d_model
        self.nhead = nhead
        self.linear = Linear(d_model, tgt_vocab_size)
        self.transformer = nn.Transformer(d_model=d_model, nhead=nhead, num_encoder_layers=num_encoder_layers,
                                       num_decoder_layers=num_decoder_layers, dim_feedforward=dim_feedforward,
                                       dropout=dropout, activation=activation)

        self.encoder_embedding = embedding_layer(d_model) #nn.Embedding(src_vocab_size, d_model)
        self._reset_parameters()  # initialize all the parameters randomly

    def _reset_parameters(self):
        # Initiate parameters in the transformer model.
        for p in self.parameters():
            if p.dim() > 1:
                xavier_uniform_(p)

    def change_to_value_net(self):
        self.linear = Linear(self.d_model, 1)  # now doing regression
        torch.nn.init.xavier_uniform(self.linear.weight)  # initialize new weights

    '''
    args:
    src_key_padding_mask: mask out padded portion of src (is (N,S))
    tgt_mask: mask out future target words (I think usually just a square triangular matrix)
    tgt_key_padding_mask: mask out padded portion of tgt
    memory: (is the encoder output) in the case of testing or policy gradients, 
              we reuse this output so want to give option to give it here
    '''

    def forward(self, src, tgt, src_key_padding_mask=None, tgt_mask=None, tgt_key_padding_mask=None,
                memory_key_padding_mask=None, memory=None, only_return_last_col=False):

        '''
        First embed src and tgt, then add positional encoding, then run encoder,
        then decoder.
        '''
        if memory is None:
            # here we reuse encoder output from previous decoding step
            memory = self.encoder_embedding(src).double() * math.sqrt(self.d_model)
            memory = self.pos_encoder(memory).double()
            memory = self.encoder(memory, src_key_padding_mask=src_key_padding_mask)

        tgt2 = self.decoder_embedding(tgt).double() * math.sqrt(self.d_model)
        tgt2 = self.pos_encoder(tgt2)

        output = self.decoder(tgt2, memory, tgt_mask=tgt_mask,
                              tgt_key_padding_mask=tgt_key_padding_mask,
                              memory_key_padding_mask=memory_key_padding_mask)

        # linear layer increases embedding dimension to size of vocab (then can feed through softmax)
        # If value_net, then linear layer will reduce embedding dim to size 1 since just regression
        output = self.linear(output)  # The cross entropy loss will take in the unnormalized outputs
        return output, memory


def generate_square_subsequent_mask(sz):
    # Generate a square mask for the sequence. The masked positions are filled with float('-inf').
    # Unmasked positions are filled with float(0.0).
    mask = (torch.triu(torch.ones(sz, sz)) == 1).float().transpose(0, 1)
    mask = mask.float().masked_fill(mask == 0, float('-inf')).masked_fill(mask == 1, float(0.0))
    return mask


class PositionalEncoding(nn.Module):

    def __init__(self, d_model, dropout=0.1, max_len=5000):
        super(PositionalEncoding, self).__init__()
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float64).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).double() * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + Variable(self.pe.transpose(0, 1)[:x.size(0), :], requires_grad=False)
        return self.dropout(x)


# now our optimizer (uses adam but changes learning rate over time)
# is in paper and http://nlp.seas.harvard.edu/2018/04/03/attention.html#optimizer
class NoamOpt:
    "Optim wrapper that implements rate."

    def __init__(self, model_size, factor, warmup, optimizer):
        self.optimizer = optimizer
        self._step = 0
        self.warmup = warmup
        self.factor = factor
        self.model_size = model_size
        self._rate = 0

    def step(self):
        "Update parameters and rate"
        self._step += 1
        rate = self.rate()
        for p in self.optimizer.param_groups:
            p['lr'] = rate
        self._rate = rate
        self.optimizer.step()

    def rate(self, step=None):
        "Implement `lrate` above"
        if step is None:
            step = self._step
        return self.factor * \
               (self.model_size ** (-0.5) *
                min(step ** (-0.5), step * self.warmup ** (-1.5)))


def get_std_opt(model):
    return NoamOpt(model.d_model, 1, 4000,
                   torch.optim.Adam(model.parameters(), lr=5e-4, betas=(0.9, 0.98), eps=1e-9))


