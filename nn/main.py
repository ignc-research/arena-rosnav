import os
import numpy as np
import ssl
import wandb
import glob
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.models as models
from pathlib import Path
from tqdm import tqdm
from typing import Union, List, Dict, Any, cast
from datetime import datetime
from torch.utils.data import (
    DataLoader,
    Dataset,
    random_split,
    WeightedRandomSampler,
)


ssl._create_default_https_context = (
    ssl._create_unverified_context  # ignore ssl certificate errors to download resnet pretrained models
)

# Check if GPU is available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
if device.type == "cuda":
    print("Cool! You have {} GPU/s".format(torch.cuda.device_count()))
else:
    print("Zero GPUs, No CUDA today :(")

print(f"torch version: {torch.__version__}")

# %% Weights & Biases

user_name = os.getlogin()

# %% Load the data

home = os.getenv("HOME")
step1 = Path(f"{home}/data/NaviPrediction/v1/")


# %% Create a custom dataset class


class Navi_Dataset(Dataset):
    def __init__(self, transform=None, scale=False):
        self.len = len([1, 2, 3]) - 1  # TODO: implement this
        self.transform = transform
        self.scale = scale  # scale x values to be in the range 0-1

    def __len__(self):
        return self.len

    def __getitem__(self, idx):
        batch = self.data[idx]  # TODO: implement this
        # Image + Metrics
        x = batch["image"]
        y = batch["label"]
        if self.transform:
            x = self.transform(x)
            # scale to 0-1
        if self.scale:
            x -= x.min(1, keepdim=True)[0]
            x /= x.max(1, keepdim=True)[0]
        return x, y

    def set_transform(self, transform):
        print(transform)
        self.transform = transform

    def n_out(self):
        return self.n_out  # TODO: implement this to return the number of classes


# Create a dataset object
navi_dataset = Navi_Dataset()

# %% Loader configuration

params = {
    "batch_size": 128,
    "shuffle": False,
    "num_workers": 6,
}

# %% Calculate mean and std of the dataset

mean = std = None


def calculate_mean_std(loader):
    loader = DataLoader(navi_dataset, **params)
    _channels_count = len(loader.dataset[0][0])
    _mean = torch.zeros(_channels_count)
    _std = torch.zeros(_channels_count)

    for images, _ in loader:
        # show progress
        print(".", end="")

        # batch size (the last batch can have smaller size!)
        batch_size = images.size(0)
        images = images.view(batch_size, images.size(1), -1)
        _mean += images.mean(2).sum(0)
        _std += images.std(2).sum(0)

    _mean /= len(loader.dataset)
    _std /= len(loader.dataset)

    print("\n")
    print(f"mean = {_mean}")
    print(f"std = {_std}")
    return _mean, _std


if mean is None:
    # Dataloader for all the dataset (train and val)
    dataloader = DataLoader(navi_dataset, **params)
    mean, std = calculate_mean_std(dataloader)

# %% Split the dataset into training and validation sets

val_percentage = 0.2  # The percentage of data to use for validation
val_count = int((len(navi_dataset) * val_percentage))
train_count = len(navi_dataset) - val_count
split_counts = [train_count, val_count]
assert sum(split_counts) == len(navi_dataset)  # sanity check

print(f"Train/Val split: {(1 - val_percentage) * 100}% / {val_percentage * 100}%")
print(f"Train count: {train_count} Val count: {val_count}")

trainset, valset = random_split(
    navi_dataset, split_counts, generator=torch.Generator().manual_seed(42)
)

# %% Create a data loaders for the training set and validation set

# TODO: Use because the date is super imbalanced, CrossEntropyLoss is too slow,
#  Using WeightedRandomSampler might help, to speed up learning.
#
# Calculate the class weights
# weights = torch.zeros(len(navi_dataset.classes))
# etc.
#
# train_sampler = WeightedRandomSampler(weights, len(trainset))
# train_loader = DataLoader(trainset, sampler=train_sampler, **params)
#
# val_sampler = WeightedRandomSampler(weights, len(valset))
# val_loader = DataLoader(valset, sampler=val_sampler, **params)

train_loader = DataLoader(trainset, **params)
val_loader = DataLoader(valset, **params)

# %% Make sure we have the correct number of channels (Bands)

# TODO: implement this
channels_count = trainset[1][0].shape[0]  # Should be 3 in this case
assert channels_count == 3  # scream at me if this is not true

# %% Show the shape first batch of training data

for data, outputs in train_loader:
    print(f"Data shape: {data.shape}, Target shape: {outputs.shape}")
    print(f"Data type: {data.dtype}, Target type: {outputs.dtype}")
    break


# %% Create a normalization transform

preprocess = transforms.Compose(
    [
        transforms.Normalize(mean=mean, std=std, inplace=False),
        # TODO: add augmentation here
    ]
)

img_before = trainset[0][0]
trainset.dataset.set_transform(preprocess)  # inject the transformer
img_after = trainset[0][0]

print(f"Max/Min before normalization: {torch.max(img_before)}/{torch.min(img_before)}")
print(f"Max/Min after normalization: {torch.max(img_after)}/{torch.min(img_after)}")

# %% Model Architecture
# Create a Resnet18


def init_model():
    _model = models.resnet18(
        num_classes=19, pretrained=False
    )  # FIXME: pass the correct number of classes
    _model.conv1 = nn.Conv2d(
        3, 64, kernel_size=7, stride=2, padding=3, bias=False
    )  # TODO: change the first convolutional layer as required to fit the data shape
    _model.to(device)
    return _model


model = init_model()
# %% Training configuration

# FIXME: based on nothing values. I pulled those values from my head.
config = {
    "lr": 0.004,
    "momentum": 0.9,
    "weight_decay": 0.0005,
    "epochs": 20,
    "checkpoint": "",
}


# %% Loss function

criterion = (
    nn.BCELoss()
)  # Binary Cross Entropy Loss # TODO: this need to be changed to a proper one (not binary) with sigmoid activation

# "Categorical Cross-Entropy loss or Softmax Loss is a Softmax activation plus a Cross-Entropy loss.
#   If we use this loss, we will train a CNN to output a probability over the C classes for each image.
#   It is used for multi-class classification.
#   What you want is multi-label classification, so you will use Binary Cross-Entropy Loss or
#   Sigmoid Cross-Entropy loss. It is a Sigmoid activation plus a Cross-Entropy loss.
#   Unlike Softmax loss it is independent for each vector component (class), meaning that the loss computed for
#   every CNN output vector component is not affected by other component values.
#   That’s why it is used for multi-label classification, where the insight of an element belonging to a certain
#   class should not influence the decision for another class.
#   Now for handling class imbalance, you can use weighted Sigmoid Cross-Entropy loss.
#   So you will penalize for wrong prediction based on the number/ratio of positive examples."
# Source: https://stackoverflow.com/questions/59336899/which-loss-function-and-metrics-to-use-for-multi-label-classification-with-very?answertab=trending#tab-top

# %% Optimizer

optimizer = torch.optim.Adam(
    model.parameters(),
    lr=config["lr"],
    weight_decay=config["weight_decay"],
)

# %% Training

epochs = config["epochs"]
valid_loss_tmp = np.inf

# Load checkpoint
if config["checkpoint"] != "":
    model.load_state_dict(torch.load(config["checkpoint"]))

run = ""  # Placeholder
# run = "train"  # Set to 'train' to train the model

configs = {}
configs.update(config)
configs.update(params)
wandb.init(
    project=f"NaviPrediction",
    notes="",
    tags=["resnet18", "experiment 0"],
    config=configs,
    entity=user_name,
)

sig = nn.Sigmoid()
if run == "train":
    for e in range(epochs):
        train_loss = 0.0
        for data, targets in tqdm(train_loader, total=len(train_loader)):
            if torch.cuda.is_available():
                data, targets = data.to(device), targets.to(device)
            optimizer.zero_grad()
            outputs = model(data)  # forward Pass
            loss = criterion(sig(outputs), targets.float())  # loss calculation
            loss.backward()  # calculate gradients
            optimizer.step()  # update Weights
            train_loss += loss.item()  # update training loss

        valid_loss = 0.0
        model.eval()
        for data, targets in val_loader:
            if torch.cuda.is_available():
                data, targets = data.to(device), targets.to(device)
            outputs = model(data)  # forward Pass
            loss = criterion(outputs.sigmoid(), targets.float())  # loss calculation

            print(f"outputs: {outputs[0]}")
            print(f"Loss: {loss.item()}")
            valid_loss += loss.item()  # update validation loss

        print(
            f"\nEpoch #{e + 1}\n",
            f"\tTrain loss: {train_loss/len(train_loader)}\n",
            f"\tValid loss: {valid_loss/len(val_loader)}\n",
        )

        wandb.log(
            {
                "epoch": e + 1,
                "train_loss": train_loss / len(train_loader),
                "valid_loss": valid_loss / len(val_loader),
            }
        )

        # if valid_loss_tmp > valid_loss:
        #     valid_loss_tmp = valid_loss  # update the minimum validation loss
        #     _dt = datetime.now().strftime("%Y-%m-%d-%H")
        #     model_path = f"./tmp/{country_name}_model_{epochs}_{_dt}.pt"
        #     torch.save(model.state_dict(), model_path)
        #     wandb.log({"model_path": model_path})
        #
        #     print(
        #         f"Model ({model_path}) saved with validation loss: {valid_loss_tmp/len(val_loader)}"
        #     )

        model_path = f"./tmp/NaviPrediction/NaviPrediction_epoch_{e+1}.pt"
        torch.save(model.state_dict(), model_path)
        wandb.log({"model_path": model_path})

        print(
            f"Model ({model_path}) saved with validation loss: {valid_loss_tmp/len(val_loader)}"
        )


# TODO: Evaluate the model, confusion matrix, etc.

# %% Utils

# Load the model from a checkpoint
def load_model(saved_file_name):
    _home = os.getenv("HOME")
    _path = Path(f"{_home}/tmp/NaviPrediction")
    saved_state_dict = torch.load(_path / saved_file_name)
    _model = init_model()
    _model.load_state_dict(saved_state_dict)
    return _model
