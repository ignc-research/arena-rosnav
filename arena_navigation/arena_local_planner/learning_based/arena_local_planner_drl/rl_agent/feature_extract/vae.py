import torch 
import torch.nn as nn
import os
import json
import numpy as np

class UnFlatten(nn.Module):
    def __init__(self, channels):
        super(UnFlatten, self).__init__()
        self.channels = channels

    def forward(self, input):
        return input.view(input.size(0), self.channels, 1, 1)
class Flatten(nn.Module):
    def forward(self, input):
        # for loading params from tf modl correctly
        input = input.permute(0,2,3,1)
        return input.reshape(input.size(0), -1)

# _-------------------------------------------------------------------------------
#DEBUG
activation = {}
def get_activation(name):
    def hook(model, input, output):
        activation[name] = output.detach().cpu().numpy()
    return hook



# _-------------------------------------------------------------------------------

class VAE(nn.Module):
    def __init__(self, image_channels=1, h_dim=1024, z_dim=32):


        super(VAE, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(image_channels, 32, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(128, 256, kernel_size=4, stride=2),
            nn.ReLU(),
            Flatten(),
        )

        self.fc1 = nn.Linear(h_dim, z_dim)
        # self.fc1.register_forward_hook(get_activation('fc1'))
        self.fc2 = nn.Linear(h_dim, z_dim)
        self.fc3 = nn.Linear(z_dim, h_dim)
        # self.fc3.register_forward_hook(get_activation('fc3'))

        self.decoder = nn.Sequential(
            UnFlatten(h_dim),
            nn.ConvTranspose2d(h_dim, 128, kernel_size=5, stride=2), 
            nn.ReLU(),
            nn.ConvTranspose2d(128, 64, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.ConvTranspose2d(64, 32, kernel_size=6, stride=2),
            nn.ReLU(),
            nn.ConvTranspose2d(32, image_channels, kernel_size=6, stride=2),
            nn.Sigmoid(),
        )


        #DEBUG
        # for name,module in self.encoder.named_children():
        #     module.register_forward_hook(get_activation('encoder'+name))
   
        # for name,module in self.decoder.named_children():
        #     module.register_forward_hook(get_activation('decoder'+name))
        

    def reparameterize(self, mu, logvar):
        std = logvar.mul(0.5).exp_()
        # return torch.normal(mu, std)
        # if self.gpu:
        #     eps = torch.cuda.FloatTensor(*mu.size()).normal_()
        # else:
        #     eps = torch.FloatTensor(*mu.size()).normal_()
        # z = mu + std * eps
        z = mu
        return z

    def bottleneck(self, h):
        mu, logvar = self.fc1(h), self.fc2(h)
        z = self.reparameterize(mu, logvar)
        return z, mu, logvar

    def encode(self, x):
        h = self.encoder(x)
        z, mu, logvar = self.bottleneck(h)
        return z, mu, logvar

    def decode(self, z):
        z = self.fc3(z)
        z = self.decoder(z)
        return z

    def forward(self, x):
        z, mu, logvar = self.encode(x)
        z = self.decode(z)
        return z, mu, logvar

    def load_from_file(self):
        json_filepath_relative = "vae.json"
        dir_path = os.path.dirname(__file__)
        json_filepath = os.path.join(dir_path, json_filepath_relative)
        print(f"loading and parsing vae params from {json_filepath}")
        with open(json_filepath,"r")  as f:
            params = json.load(f)
        with torch.no_grad():
            # parse encoder
            for i in range(4):
                weight_np = np.transpose(np.array(params[i*2],dtype=np.float32),(3,2,0,1))/10000.0
                bias_np = np.array(params[i*2+1],dtype=np.float32)/10000.0
                conv_layer= self.encoder[i*2]
                conv_layer.weight = nn.Parameter(torch.from_numpy(weight_np).float())
                conv_layer.bias = nn.Parameter(torch.from_numpy(bias_np).float())

            # mu log
            weight_np = np.array(params[4*2],dtype=np.float32).T/10000.0
            bias_np = np.array(params[4*2+1],dtype=np.float32).T/10000.0
            self.fc1.weight = nn.Parameter(torch.from_numpy(weight_np).float())
            self.fc1.bias = nn.Parameter(torch.from_numpy(bias_np).float())
            # logvar
            weight_np2 = np.array(params[5*2],dtype=np.float32).T/10000.0
            bias_np2 = np.array(params[5*2+1],dtype=np.float32).T/10000.0
            self.fc2.weight = nn.Parameter(torch.from_numpy(weight_np2).float())
            self.fc2.bias = nn.Parameter(torch.from_numpy(bias_np2).float())
            
            # logvar
            weight_np3 = np.array(params[6*2],dtype=np.float32).T/10000.0
            bias_np3 = np.array(params[6*2+1],dtype=np.float32).T/10000.0
            self.fc3.weight = nn.Parameter(torch.from_numpy(weight_np3).float())
            self.fc3.bias = nn.Parameter(torch.from_numpy(bias_np3).float())



            # This part is incorrect, but ohters part have been validated, and means encoding part works! YEAH!
            # decoder un_flatten
            for i in range(7,7+4):
                if i == 7:
                    weight_np = np.array(params[i*2],dtype=np.float32)/10000.0
                    layer = self.decoder[0]
                else:
                    weight_np = np.transpose(np.array(params[i*2],dtype=np.float32),(3,2,0,1))/10000.0
                    layer = self.decoder[(i-7)*2+1]
                bias_np = np.array(params[i*2+1],dtype=np.float32)/10000.0
                layer.weight = nn.Parameter(torch.from_numpy(weight_np).float())
                layer.bias = nn.Parameter(torch.from_numpy(bias_np).float())



            
if __name__ == '__main__':
    from ring import generate_rings
    from matplotlib import pyplot as plt
    vae = VAE()
    vae.load_from_file()


    # ring_def = generate_rings(expansion_term=0,min_dist=0.1,max_dist=3.5,VISUALIZE = True)
    # # linear ramp
    # scans = np.ones((1, 360)) * (np.arange(360) / 360.0 * 3.5)[None, :]

    # rings_ori = ring_def["lidar_to_rings"](scans.astype(np.float32))/ring_def["rings_to_bool"]

    # rings_new,_,_ = vae.forward(torch.tensor(rings_ori.reshape([1,1,64,64]).astype(np.float32)).cuda())
    # rings_new *= ring_def["rings_to_bool"]
    # ring_def["visualize_rings"](rings_ori[0, :, :, :], scan=scans[0])
    # plt.savefig("test_org.png")
    # ring_def["visualize_rings"](rings_new[0, 0, :, :].cpu().detach().numpy()[...,None], scan=scans[0]) 
    # plt.savefig("test_new.png")



    ring_def = generate_rings()
    # linear ramp
    scans = np.ones((1, 1080)) * (np.arange(1080) / 1080.0 * 25.0)[None, :]

    rings_ori = ring_def["lidar_to_rings"](scans.astype(np.float32))/ring_def["rings_to_bool"]

    rings_new,_,_ = vae.forward(torch.tensor(rings_ori.reshape([1,1,64,64]).astype(np.float32)).cuda())
    # plt.figure("training_status")
    # plt.clf()
    # f, (ax1, ax2) = plt.subplots(2, 1, num="training_status")
    # ax1.imshow(rings_ori[0, :, :, 0], cmap=plt.cm.Greys)
    # ax2.imshow(rings_new[0, 0, :, :].cpu().detach().numpy(), cmap=plt.cm.Greys)
    # plt.savefig("compare2.png")
    pass

    np.savez("th",**activation)
        


    # rings_new *= ring_def["rings_to_bool"]
    # ring_def["visualize_rings"](rings_ori[0, :, :, :], scan=scans[0])
    # plt.savefig("test_org_2.png")
    # ring_def["visualize_rings"](rings_new[0, 0, :, :].cpu().detach().numpy()[...,None], scan=scans[0]) 
    # plt.savefig("test_new_2.png")

                

