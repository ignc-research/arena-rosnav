import os
import re
import seaborn as sns
import pandas as pd
import yaml
import torch
import numpy as np
import torchvision
import argparse as ap
import wandb as wandb
from PIL import Image
from matplotlib import pyplot as plt
from torch import nn
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
        # This is a list of performance metrics to use as targets
        # In this case, we can use success_rate, collision_rate, and episode_duration, or any combination of them.
        # NOTE: The order of the list is important, as it determines the order of the targets in the output tensor.
        target_metrics: list = None,
    ):
        self.mean_std_path = data_root / "mean_std_cache.yaml"
        if target_metrics is None:
            target_metrics = ["success_rate"]
        self.mean_std = None
        assert type(data_root) in [PosixPath], f"Source type {type(data_root)} not supported"
        # target_metrics must be a subset of [success_rate, collision_rate, episode_duration]
        assert len(target_metrics) > 0, "target_metrics must have at least one element"
        assert all(
            [metric in ["success_rate", "collision_rate", "episode_duration"] for metric in target_metrics]
        ), "target_metrics must be a subset of [success_rate, collision_rate, episode_duration]"

        self.target_metrics = target_metrics
        self.source = data_root
        self.img_transform = img_transform
        self.meta_transform = meta_transform
        self.log = _log or new_logger()

        # get all directories in data_root
        all_dirs = [d for d in data_root.iterdir() if d.is_dir()]

        # Count number of directories in data_root directory and set length
        # Match UUID4 directories
        uuid4_pattern = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[4][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$")

        # delete directories that do not match UUID4 pattern
        dirs = [d for d in all_dirs if uuid4_pattern.match(d.name) is not None]

        self.dirs = dirs
        self.len = len(self.dirs)

        self.log.info(f"Found {self.len} datapoints in {data_root}")

        # load mean and std from cache
        if self.mean_std_path.exists():
            with open(self.mean_std_path, "r") as f:
                self.mean_std = yaml.safe_load(f)
        else:
            self.log.info("No mean and std cache found, calculating...")
            self.mean_std = self.calc_mean_std()
            self.log.info("Done calculating mean and std")

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

            # scale to [0, 1]
            meta = (meta - self.mean_std["meta_min"]) / (self.mean_std["meta_max"] - self.mean_std["meta_min"])

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

        performance_metrics_dict = meta_dict["performance_metrics"]

        # keep success_rate in performance_metrics and remove everything else
        performance_metrics = [performance_metrics_dict[metric] for metric in self.target_metrics]

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
                    inplace=True,
                ),
                # random vertical flip
                transforms.RandomVerticalFlip(p=0.5),
                # random horizontal flip
                transforms.RandomHorizontalFlip(p=0.5),
            ]
        )

        self.meta_transform = transforms.Compose(
            [
                transforms.Normalize(
                    mean=torch.tensor(self.mean_std["meta_mean"]).unsqueeze(0).unsqueeze(0),
                    std=(torch.tensor(self.mean_std["meta_std"]) + 1e-8)  # add small value to avoid division by zero
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

        results = {
            "img_mean": img_mean.tolist(),
            "img_std": img_std.tolist(),
            "meta_mean": meta_mean.tolist(),
            "meta_std": meta_std.tolist(),
            "meta_max": meta_max.item(),
            "meta_min": meta_min.item(),
        }

        # save mean and std to yaml file
        with open(self.mean_std_path, "w") as f:
            yaml.dump(results, f)

        return results


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
                dropout=0.1,
            ),
            num_classes=None,
            dropout=0.1,
            post_emb_norm=False,
            emb_dropout=0.1,
        ).to(device)

        self.meta_encoder = Encoder(
            dim=55,
            depth=6,
            heads=8,
            dropout=0.1,
        ).to(device)

        # sigmoid activation for collision_rate and success_rate
        self.sigmoid = torch.nn.Sigmoid()

        self.fc = nn.Sequential(
            nn.TransformerEncoderLayer(d_model=155, nhead=31, batch_first=True, dropout=0.1),
            nn.Linear(155, 100),
            nn.ReLU(),
            nn.TransformerEncoderLayer(d_model=100, nhead=10, batch_first=True, dropout=0.1),
            nn.Linear(100, 100),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.TransformerEncoderLayer(d_model=100, nhead=10, batch_first=True, dropout=0.1),
            nn.Linear(100, 80),
            nn.ReLU(),
            nn.TransformerEncoderLayer(d_model=80, nhead=10, batch_first=True, dropout=0.1),
            nn.Linear(80, 70),
            nn.ReLU(),
            nn.TransformerEncoderLayer(d_model=70, nhead=10, batch_first=True, dropout=0.1),
            nn.Linear(70, 60),
            nn.ReLU(),
            nn.TransformerEncoderLayer(d_model=60, nhead=10, batch_first=True, dropout=0.1),
            nn.Linear(60, 40),
            nn.ReLU(),
            nn.TransformerEncoderLayer(d_model=40, nhead=10, batch_first=True),
            nn.Linear(40, 1),
            nn.TransformerEncoderLayer(d_model=1, nhead=1, batch_first=True),
        ).to(device)
        # fully connected layers sequence for mixed input

    def forward(self, image: torch.Tensor, metadata: torch.Tensor) -> torch.Tensor:
        # To flatten the image tensor use the following line
        # img = image.view(image.size(0), -1)

        img = self.encoder(image)
        meta = self.meta_encoder(metadata.unsqueeze(1))

        mix = torch.cat((img, meta.squeeze(1)), dim=1)  # (100 + 20) concatenate image and metadata
        output = self.fc(mix)

        return output


# %% Criterion


class RMSELoss(torch.nn.Module):
    def __init__(self):
        super(RMSELoss, self).__init__()

    def forward(self, x, y):
        mseloss = nn.MSELoss()
        eps = 1e-6  # to avoid division by zero
        loss = torch.sqrt(mseloss(x, y) + eps)
        return loss


def criterion():
    return RMSELoss()


# %% Evaluation Metrics
class Evaluator:
    def __init__(
        self,
        criterion=None,
        device: torch.device = None,
    ):
        self.mse = 0.0  # mean squared error
        self.rmse = 0.0  # root mean squared error
        self.val_loss = 0.0  # loss
        self.update_count = 0
        self.device = device
        self.criterion = criterion

        assert self.criterion is not None, "Criterion not set"
        assert self.device is not None, "Device not set"

        self._mse = torch.nn.MSELoss()  # Mean Squared Error (MSE).
        self._rmse = RMSELoss()  # Root Mean Squared Error (RMSE).
        self._precision = Precision().to(self.device)
        self._recall = Recall().to(self.device)

    def update(self, preds: torch.Tensor, _target: torch.Tensor):
        self.mse += self._mse(preds, _target).item()
        self.rmse += self._rmse(preds, _target).item()
        self.val_loss += self.criterion(preds.sigmoid(), _target.float()).item()

        self.update_count += 1

    def __str__(self) -> str:
        return (
            f"\tMSE: {self.mse / self.update_count :.4f}\n"
            f"\tRMSE: {self.rmse / self.update_count :.4f}\n"
            f"\tValidation Loss: {self.val_loss / self.update_count :.4f}\n"
        )

    def get(self) -> Dict[str, float]:
        return {
            "mse": self.mse / self.update_count,
            "rmse": self.rmse / self.update_count,
            "val_loss": self.val_loss / self.update_count,
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
        bn = 0  # number of batches
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

            if bn % 10 == 0:
                batch_size = img.shape[0]  # get the batch size
                random_sample = np.random.randint(0, batch_size)
                log.info(f"target: {target[random_sample].detach().cpu().numpy()[0]:.2f}")
                log.info(f"  pred: {_pred[random_sample].detach().cpu().numpy()[0]:.2f}")
                log.info(
                    f" error: {target[random_sample].detach().cpu().numpy()[0] - _pred[random_sample].detach().cpu().numpy()[0]:.2f}"
                )

            loss = _criterion(_pred, target)
            loss.backward()  # Compute the gradients
            _optimizer.step()  # Update the weights
            train_loss += loss.item()  # Accumulate the loss
            bn += 1  # Increment the batch number

        # Validation
        __metrics = _evaluator.evaluate(_model, _val_loader)

        log.debug(f"\n\nMetrics:\n" f"\tTrainLoss: {train_loss / len(_train_loader):.8f}\n" f"{__metrics}")

        _logs = {
            f"train_loss": train_loss / len(_train_loader),
        }

        for k, v in __metrics.get().items():
            _logs[f"{k}"] = v

        log.info(_logs)
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
    # --train and --analyze are mutually exclusive
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--train", action="store_true", help="Train model")
    group.add_argument("--analyze", action="store_true", help="Analyze data")
    parser.add_argument("--data_root", help="Path to data root", type=str, default=str(cwd / "data"))
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
    parser.add_argument("--lr", help="Learning rate", type=float, default=4e-3)
    parser.add_argument("--num_workers", help="Number of workers", type=int, default=0)
    parser.add_argument(
        "--checkpoint_dir",
        help="Checkpoint directory",
        type=str,
        default=str(cwd / "checkpoints"),
    )

    args = parser.parse_args()
    args.data_root = Path(args.data_root)

    assert args.data_root.is_dir(), "data_root must be a directory"

    # %% Logging

    log = new_logger(
        level=args.log_level,
        module_name=f"{__name__}".replace("__", ""),
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

    if args.train:
        log.info("=== Training ===")
        train(
            _model=NavModel(),
            _train_loader=train_loader,
            _val_loader=val_loader,
            _criterion=criterion(),
            _wandb_run=run,
            _device=device,
            _epochs=args.epochs,
        )
    # %% Start analysis
    if args.analyze:
        log.info("=== Analysis ===")
        local_target_metrics = ["success_rate", "collision_rate", "episode_duration"]
        train_set = CustomDataset(
            data_root=args.data_root / "train",
            _log=log,
            target_metrics=local_target_metrics,
        )
        # Plot the distribution of success_rates
        metrics = [x[2].numpy() for x in train_set]

        # success_rates as pd dataframe
        df = pd.DataFrame(metrics, columns=local_target_metrics)
        df["success_rate"] = df["success_rate"].apply(lambda x: round(x, 2))
        df["collision_rate"] = df["collision_rate"].apply(lambda x: round(x, 2))
        df["episode_duration"] = df["episode_duration"].apply(lambda x: round(x, 2))

        # %%

        def plot_distribution(_df, col, title, xlim=None):
            color_palette = sns.color_palette("rocket_r", 5)
            # Plot the distribution of success_rates
            ax = plt.gca()
            ax.grid(False)

            # set x-axis min and max
            if xlim:
                ax.set_xlim(xlim)

            # Plot KDE on top of histogram
            sns.set_theme(style="whitegrid")

            xlabel = col.replace("_", " ").title()
            ax.set_xlabel(xlabel)
            ax.set_yticklabels([])
            ax.set_ylabel("")
            ax.set_title(title)
            ax.tick_params(left=False, bottom=False)
            ax.spines.left.set_visible(False)
            sns.histplot(
                _df[col],
                ax=ax,
                alpha=0.6,
                bins=18,
                kde=True,
                color=color_palette[0],
            )

            # set plot size
            fig = plt.gcf()
            fig.set_size_inches(8, 8)

            # Calculate percentiles and annotate plot
            q = [0.2, 0.5, 0.8]

            # Plot percentiles as vertical lines on the histogram for each percentile
            for i in q:
                ax.axvline(
                    _df[col].quantile(i),
                    linestyle=":",
                    alpha=0.5,
                    color=color_palette[2],
                )
                ax.text(
                    _df[col].quantile(i),
                    # set height of text to 0.5 times the height of the histogram
                    (ax.get_ylim()[1] * 0.1) * i,
                    f"{i*10:.0f}th pctl",
                    color=color_palette[1],
                    horizontalalignment="center",
                    verticalalignment="bottom",
                )

            # Remove spines
            for s in ["top", "right"]:
                ax.spines[s].set_visible(False)

            plt.show()

        plot_distribution(df, "success_rate", "Distribution of success rates".title(), xlim=(0, 1))
        plot_distribution(df, "collision_rate", "Distribution of collision rates".title(), xlim=(0, 1))
        plot_distribution(df, "episode_duration", "Distribution of episode durations".title())
