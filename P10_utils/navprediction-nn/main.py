import os
import re
import sys
import yaml
import torch
import numpy as np
import torchvision
import argparse as ap
import wandb as wandb
from PIL import Image
from tqdm.auto import tqdm
from util import new_logger
from typing import Union, Dict
from pathlib import Path, PosixPath
from torchmetrics import Precision, Recall
from torchvision.transforms import transforms
from torch.utils.data import DataLoader, Dataset
from x_transformers import ViTransformerWrapper, Encoder


# %% Dataset


class CustomDataset(Dataset):
    def __init__(
        self,
        data_root: Union[PosixPath, Path] = None,
        img_transform=None,
        meta_transform=None,
        _log=None,
    ):
        self.mean_std = None
        assert type(data_root) in [PosixPath], f"Source type {type(data_root)} not supported"

        self.source = data_root
        self.img_transform = img_transform
        self.meta_transform = meta_transform
        self.log = _log or new_logger()

        # get all directories in data_root
        all_dirs = [d for d in data_root.iterdir() if d.is_dir()]

        # Count number of directories in data_root directory and set length
        # Match UUID4 directories
        uuid4_pattern = re.compile(
            r"^[0-9a-f]{8}-[0-9a-f]{4}-[4][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$"
        )

        # delete directories that do not match UUID4 pattern
        dirs = [d for d in all_dirs if uuid4_pattern.match(d.name) is not None]

        self.dirs = dirs
        self.len = len(self.dirs)

        self.log.info(f"Found {self.len} datapoints in {data_root}")
        self.mean_std = self.calc_mean_std()

    def __len__(self):
        return self.len

    def __getitem__(self, idx):
        datapoint = self.load_datapoint(idx)
        img = datapoint["image"]
        meta = datapoint["metadata"]
        target = datapoint["target"]
        if self.img_transform is not None:
            img = img.unsqueeze(0)  # add channel dimension
            img = self.img_transform(img)  # apply image transform
            img /= 255.0  # scale to [0, 1]
        if self.meta_transform is not None:
            # Using torch transform to normalize metadata torch transform expects an image tensor as input,
            # but we have a vector unsqueeze twice to fake an image like tensor then squeeze twice to get back to
            # vector shape. It's a bit of a hack, but it works.
            meta = meta.unsqueeze(0).unsqueeze(0)
            meta = self.meta_transform(meta)
            meta = meta.squeeze(0).squeeze(0)
            meta -= self.mean_std["meta_min"]
            meta /= self.mean_std["meta_max"]

        return img, meta, target

    def load_datapoint(self, idx: int) -> Dict[str, torch.Tensor]:
        _dir = self.dirs[idx]

        # load png images from batch_dirs
        img = self.load_image(_dir)

        # convert PIL.Image to grayscale image
        img = img.convert("L")

        # convert PIL.Image to torch.Tensor
        image = transforms.ToTensor()(img)

        # load metadata from yaml files
        meta_dict = self.load_metadata(_dir)

        other_metadata = list(meta_dict["other_metadata"].values())
        performance_metrics = list(meta_dict["performance_metrics"].values())

        # concatenate performance_metrics and other_metadata
        other_metadata = torch.from_numpy(np.array(other_metadata, dtype=np.float32))
        performance_metrics = torch.from_numpy(np.array(performance_metrics, dtype=np.float32))

        return {
            "image": image,
            "metadata": other_metadata,
            "target": performance_metrics,
        }

    def init_transformers(self):

        self.img_transform = transforms.Compose(
            [
                transforms.Normalize(
                    mean=self.mean_std["img_mean"],
                    std=self.mean_std["img_std"],
                    inplace=False,
                ),
                # random vertical flip
                transforms.RandomVerticalFlip(p=0.5),
                # random horizontal flip
                transforms.RandomHorizontalFlip(p=0.5),
                # random rotation
                transforms.RandomRotation(degrees=(-5, 5)),
            ]
        )

        self.meta_transform = transforms.Compose(
            [
                transforms.Normalize(
                    mean=torch.tensor(self.mean_std["meta_mean"]).unsqueeze(0).unsqueeze(0),
                    std=(
                        torch.tensor(self.mean_std["meta_std"])
                        + 1e-8  # add small value to avoid division by zero
                    )
                    .unsqueeze(0)
                    .unsqueeze(0),
                    inplace=False,
                ),
            ]
        )

    @staticmethod
    def load_image(_dir: PosixPath) -> Image:
        # extract image name from directory name
        image_name = _dir.name
        # load png image from d
        return Image.open(_dir / f"{image_name}_img.png")

    @staticmethod
    def load_metadata(_dir: PosixPath) -> dict:
        # extract yaml files from directory
        yaml_files = [f for f in _dir.iterdir() if f.suffix == ".yml"]

        # delete yaml files that do not start with the directory name
        yaml_files = [f for f in yaml_files if f.name.startswith(_dir.name)]

        # load yaml files as a dict
        metadata = [yaml.load(open(f, "r"), Loader=yaml.SafeLoader) for f in yaml_files]

        # merge metadata inner dicts
        metadata = {k: v for d in metadata for k, v in d.items()}

        # extract performance metrics from metadata
        performance_metrics = ["collision_rate", "episode_duration", "success_rate"]
        performance_metrics_dict = {k: v for k, v in metadata.items() if k in performance_metrics}

        # Get other metadata from metadata
        other_metadata = [k for k in metadata.keys() if k not in performance_metrics]
        other_metadata_dict = {k: v for k, v in metadata.items() if k in other_metadata}

        # return performance metrics and other metadata
        return {
            "performance_metrics": performance_metrics_dict,
            "other_metadata": other_metadata_dict,
        }

    def calc_mean_std(self):
        _loader = DataLoader(self, batch_size=128, shuffle=False)
        img_channels_count = 1
        img_mean = torch.zeros(img_channels_count)
        img_std = torch.zeros(img_channels_count)

        metadata_params_count = _loader.dataset[0][1].shape[0]
        meta_mean = torch.zeros(metadata_params_count)
        meta_std = torch.zeros(metadata_params_count)
        meta_max = None
        meta_min = None

        for img, meta, _ in tqdm(_loader, desc="Calculating mean and std", unit="batch"):
            # calculate mean and std of an image batch
            img_mean += img.mean().sum()
            img_std += img.std().sum()

            # calculate mean and std for each element in metadata
            meta_mean += meta.mean(0)  # mean across batch dimension
            meta_std += meta.std(0)  # std across batch dimension

            # calculate max and min for each element in metadata
            if meta_max is None:
                meta_max = meta.max()
                meta_min = meta.min()
            else:
                # update max and min
                meta_max = torch.max(meta_max, meta.max())
                meta_min = torch.min(meta_min, meta.min())

        img_mean /= self.len
        img_std /= self.len

        meta_mean /= self.len
        meta_std /= self.len

        return {
            "img_mean": img_mean.tolist(),
            "img_std": img_std.tolist(),
            "meta_mean": meta_mean.tolist(),
            "meta_std": meta_std.tolist(),
            "meta_max": meta_max.item(),
            "meta_min": meta_min.item(),
        }


# %% Dummy Model
class NavModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.name = "NavModel"

        self.encoder = ViTransformerWrapper(
            image_size=150,
            patch_size=5,
            channels=1,
            attn_layers=Encoder(
                dim=100,
                depth=3,
                heads=12,
            ),
            num_classes=None,
            dropout=0.0,
            post_emb_norm=False,
            emb_dropout=0.0,
        ).to(device)

        self.meta_layer_1 = torch.nn.Linear(55, 35)
        self.meta_layer_2 = torch.nn.Linear(35, 25)
        self.meta_layer_3 = torch.nn.Linear(25, 20)

        self.mix_layer_1 = torch.nn.Linear(120, 60)
        self.mix_layer_2 = torch.nn.Linear(60, 3)

    def forward(self, image: torch.Tensor, metadata: torch.Tensor) -> torch.Tensor:
        # To flatten the image tensor use the following line
        # img = image.view(image.size(0), -1)

        img = self.encoder(image)

        meta = metadata.view(metadata.size(0), -1)  # flatten metadata
        meta = self.meta_layer_1(meta)
        meta = torch.nn.functional.relu(meta)
        meta = self.meta_layer_2(meta)
        meta = torch.nn.functional.relu(meta)
        meta = self.meta_layer_3(meta)

        mix = torch.cat((img, meta), dim=1)  # (100 + 20) concatenate image and metadata

        mix = self.mix_layer_1(mix)
        mix = torch.nn.functional.relu(mix)
        mix = self.mix_layer_2(mix)

        return mix


# %% Evaluation Metrics


class Evaluator:
    def __init__(
        self,
        criterion=None,
        device: torch.device = None,
    ):
        self.mse = 0.0  # mean squared error
        self.loss = 0.0  # loss
        self.update_count = 0
        self.device = device
        self.criterion = criterion

        assert self.criterion is not None, "Criterion not set"
        assert self.device is not None, "Device not set"

        self._mse = torch.nn.MSELoss()  # Mean Squared Error (MSE).
        self._precision = Precision().to(self.device)
        self._recall = Recall().to(self.device)

    def update(self, preds: torch.Tensor, _target: torch.Tensor):
        self.mse += self._mse(preds, _target).item()
        self.loss += self.criterion(preds.sigmoid(), _target.float()).item()

        self.update_count += 1

    def __str__(self) -> str:
        return (
            f"\tMSE: {self.mse / self.update_count :.4f}\n"
            f"\tValLoss: {self.loss / self.update_count:.4f}\n"
        )

    def get(self) -> Dict[str, float]:
        return {
            "mse": self.mse / self.update_count,
            "val_loss": self.loss / self.update_count,
        }

    def evaluate(self, _model, _loader):
        _model.eval()  # set model to evaluation mode
        _metrics = self
        for img, meta, target in tqdm(
            _loader,
            total=len(_loader),
            desc="Evaluating",
            unit="batch",
            dynamic_ncols=True,
        ):
            img = img.type(torch.float32).to(device)
            meta = meta.type(torch.float32).to(device)
            target = target.type(torch.float32).to(device)
            preds = _model(img, meta)
            _metrics.update(preds, target)
        return _metrics


# %% Optimizer
def optimizer(_model: torch.nn.Module) -> torch.optim.Optimizer:
    """
    AdamW is a stochastic optimization method that modifies the typical implementation of weight decay in Adam,
    by decoupling weight decay from the gradient update.
    source: https://paperswithcode.com/method/adamw#:~:text=AdamW
    """
    return torch.optim.AdamW(_model.parameters(), lr=config["lr"])


# %% Criterion
def criterion():
    # HuberLoss
    return torch.nn.HuberLoss(reduction="mean")


# %% Training Function
def train(
    _criterion,
    _wandb_run,
    _epochs: int,
    _device: torch.device,
    _model: torch.nn.Module,
    _val_loader: DataLoader,
    _train_loader: DataLoader,
):
    _evaluator = Evaluator(criterion=_criterion, device=_device)
    _model.to(_device)

    for _epoch in tqdm(range(_epochs), desc=f"Epochs", unit="epoch", dynamic_ncols=True):
        # Training
        _model.train()  # set model to training mode
        train_loss = 0.0
        for img, meta, target in tqdm(
            _train_loader,
            total=len(_train_loader),
            desc=f"Training",
            unit="Batch",
            dynamic_ncols=True,
        ):
            img = img.type(torch.float32).to(_device)
            meta = meta.type(torch.float32).to(_device)
            target = target.type(torch.float32).to(_device)

            _optimizer = optimizer(_model)
            _optimizer.zero_grad()
            _pred = _model.forward(img, meta)  # forward pass
            loss = _criterion(_pred, target)
            loss.backward()  # Compute the gradients
            _optimizer.step()  # Update the weights
            train_loss += loss.item()  # Accumulate the loss

        # Validation
        __metrics = _evaluator.evaluate(_model, _val_loader)

        log.debug(
            f"\n\nMetrics:\n" f"\tTrainLoss: {train_loss / len(_train_loader):.8f}\n" f"{__metrics}"
        )

        _logs = {
            f"train_loss": train_loss / len(_train_loader),
        }

        for k, v in __metrics.get().items():
            _logs[f"{k}"] = v

        _wandb_run.log(_logs, step=_epoch)

        # Save model checkpoint every 10 epochs
        if _epoch % 10 == 0:
            # mkdir if not exists
            if not os.path.exists(config["checkpoint_dir"]):
                os.makedirs(config["checkpoint_dir"])
            torch.save(_model.state_dict(), f"{config['checkpoint_dir']}/model_{_epoch}.pth")


# %% Execution starts here
if __name__ == "__main__":
    # Get current working directory
    cwd = Path(__file__).parent

    # %% Parse arguments
    parser = ap.ArgumentParser(description="Navigation prediction experiment")
    parser.add_argument("--train", action="store_true", help="Train model")
    parser.add_argument(
        "--data_root", help="Path to data root", type=str, default=str(cwd / "data")
    )
    parser.add_argument(
        "--log_level",
        help="Log level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    )
    parser.add_argument("--random_seed", help="Random seed", type=int, default=42)
    parser.add_argument("--batch_size", help="Batch size", type=int, default=128)
    parser.add_argument("--epochs", help="Number of epochs", type=int, default=10)
    parser.add_argument("--lr", help="Learning rate", type=float, default=3e-4)
    parser.add_argument("--num_workers", help="Number of workers", type=int, default=0)
    parser.add_argument(
        "--checkpoint_dir",
        help="Checkpoint directory",
        type=str,
        default=str(cwd / "checkpoints"),
    )

    args = parser.parse_args()
    args.data_root = Path(args.data_root)

    assert args.train or args.test, "Either train or test must be specified"
    assert args.data_root.is_dir(), "data_root must be a directory"

    # %% Logging

    log = new_logger(
        module_name=f"{__name__}".replace("__", ""),
        level=args.log_level,
        stream=sys.stdout,
    )

    log.info(f"Arguments: {args}")
    log.info(f"Current working directory: {cwd}")
    log.info(f"torch version: {torch.__version__}")
    log.info(f"torchvison version: {torchvision.__version__}")

    # %% torch setup
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        raise Exception("CUDA not available")

    log.info(f"Device: {device}")

    # set random seed for reproducibility
    torch.manual_seed(args.random_seed)
    torch.cuda.manual_seed_all(args.random_seed)
    np.random.seed(args.random_seed)

    log.info(f"Random seed: {args.random_seed}")

    # %% Dataset

    train_set = CustomDataset(data_root=args.data_root / "train", _log=log)
    val_set = CustomDataset(data_root=args.data_root / "val", _log=log)

    # %% Configurations

    params = {
        "batch_size": args.batch_size,
        "shuffle": True,
        "num_workers": args.num_workers,
    }

    # TODO: based on nothing lr and momentum. I pulled those values out of my *head*.
    config = {
        "lr": args.lr,
        "epochs": args.epochs,
        "seed": args.random_seed,
        "checkpoint_dir": args.checkpoint_dir,
    }

    # %% wandb setup

    run = wandb.init(project=f"Navprediction", entity="amer")
    run.name = f"{run.name}-{args.epochs}epochs"
    run.config.update(config)
    run.config.update(params)

    # %% Create dataloaders

    train_loader = DataLoader(train_set, **params)
    val_loader = DataLoader(val_set, **params)

    # %% Start training

    train(
        _model=NavModel(),
        _train_loader=train_loader,
        _val_loader=val_loader,
        _criterion=criterion(),
        _wandb_run=run,
        _device=device,
        _epochs=args.epochs,
    )
