import torch
import torch.nn as nn
from torch.nn.utils.rnn import PackedSequence

###parameters of gru##########
embedding_size_first = 128
#embedding_size_second = 128             #number of nodes in embedding layer of the fc network
HIDDEN_SHAPE_SECOND = 96
hidden_dim = 96                          #number of nodes in GRU  
layer_dim = 2                            #number of gru layers 1 3 5 7
##############################

class GRUModel(nn.Module):
    def __init__(self, input_dim,n_actions, bias=True):
        super(GRUModel, self).__init__()
        # Hidden dimensions
        self.hidden_dim = hidden_dim
        # Number of hidden layers
        self.layer_dim = layer_dim
        self.embedding = nn.Sequential(
                                       nn.Linear(input_dim, embedding_size_first),
                                       nn.LeakyReLU(0.01),                                       
                                       nn.Linear(embedding_size_first, HIDDEN_SHAPE_SECOND))
        #self.embedding = nn.Linear(input_dim,embedding_size)	
        self.gru = nn.GRU(HIDDEN_SHAPE_SECOND, hidden_dim, layer_dim, dropout=0.1) #GRUCell(input_dim, hidden_dim, layer_dim) , dropout=0.1
        self.fc = nn.Sequential(
                        nn.LeakyReLU(0.01),
                        nn.Linear(hidden_dim, n_actions))    #nn.Linear(hidden_dim, HIDDEN_SHAPE), nn.ReLU(),
	    
    def forward(self, x:PackedSequence, h):
        embedding=self.embedding(x.data)
        embedding = PackedSequence(embedding, x.batch_sizes, x.sorted_indices, x.unsorted_indices)
        out, h = self.gru(embedding,h)        
        out = self.fc(h[-1,:,:])	
        return out,h.data

    def init_hidden(self, batch_size):
        weight = next(self.parameters()).data
        hidden = weight.new(self.layer_dim, batch_size, self.hidden_dim).zero_().cuda()
        return hidden


