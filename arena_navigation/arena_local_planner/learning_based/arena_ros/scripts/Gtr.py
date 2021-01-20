import vanillaTransformer as vt
import torch.nn as nn
import torch
import math
from torch.autograd import Variable

output_features=256
embedding_size=256              # number of output neurons of the embedding layer
HIDDEN_SHAPE_1 = 256
HIDDEN_SHAPE_2 = 256
class CartPoleEmbedder(nn.Module):
    def __init__(self,input_size,dropout=0.1, B=1, embedding_size=output_features):
        '''
        :param B: Number of times we embed each state (with dropout each time)

        '''

        super(CartPoleEmbedder, self).__init__()
        self.B = B
        self.dropout_p = dropout
        self.embedding = nn.Sequential(nn.Linear(input_size, HIDDEN_SHAPE_1),
                                       nn.ReLU(), 
                                       nn.Linear(HIDDEN_SHAPE_1, embedding_size))

        self.layer4 = nn.Linear(input_size,embedding_size)

        #now need to combine the B copies of the elements
        #Can start by using just linear combo then move to nonlinear combo

    def forward(self,input,is_training=True):
        #want to now stack B copies of input on top of eachother
        #Batch dim is dim 0
        input = torch.cat(self.B*[input])

        return self.embedding(input)



class TransformerDqn(nn.Module):

    def __init__(self,output_size,input_size,num_encoder_layers=3):
        '''
        :param embedder: module to embed the states
        :param output_size: number of actions we can choose
        '''

        #dropout = 0.1
        self.hidden_size=output_features
        self.d_model=output_features
        super(TransformerDqn, self).__init__()
        self.embedder = CartPoleEmbedder(input_size=input_size)
        self.pos_encoder = PositionalEncoding(d_model=output_features, dropout=0.1)
        self.encoder_layer = vt.StableTransformerLayer(d_model=output_features,nhead=4,dim_feedforward=128, dropout=0.1, use_gate = False)
        self.encoder = vt.TransformerEncoder(encoder_layer=self.encoder_layer,num_layers=num_encoder_layers)
        self.output_layer = nn.Linear(self.hidden_size,output_size)

    def forward(self,input):
        '''
        :param input: matrix of state vectors (last column will contain state of interest)
        :return: vector of Q values for each action
        '''       

        embedding = self.embedder(input)
        embedding = self.pos_encoder(embedding)       
        embedding = self.encoder(embedding)
        #print('embedding.size:',embedding.shape)
        #print('output.size:',self.output_layer(embedding).shape)
        return self.output_layer(embedding).squeeze(1)

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

def generate_square_subsequent_mask(sz):
    # Generate a square mask for the sequence. The masked positions are filled with float('-inf').
    # Unmasked positions are filled with float(0.0).
    mask = (torch.triu(torch.ones(sz, sz)) == 1).float().transpose(0, 1)
    mask = mask.float().masked_fill(mask == 0, float('-inf')).masked_fill(mask == 1, float(0.0))
    return mask


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


def get_std_opt(model,lr):
    return NoamOpt(model.d_model, 1, 4000,
                   torch.optim.Adam(model.parameters(), lr=lr, betas=(0.9, 0.98), eps=1e-9))




