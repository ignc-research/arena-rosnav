import PIL
import torch
from unittest import TestCase
from pathlib import Path, PosixPath
from torch.utils.data import DataLoader
from main import CustomDataset, NavModel

# get current working directory
cwd = Path(__file__).parent
val_dir = cwd / "fixtures" / "data" / "val"


def custom_dataset():
    assert val_dir.exists(), "Data directory does not exist"
    return CustomDataset(data_root=PosixPath(val_dir))


class TestMain(TestCase):
    def test_init(self):
        self.assertTrue(isinstance(custom_dataset(), CustomDataset))
        assert custom_dataset().len > 0, "Dataset is not empty"

    def test_load_datapoint(self):
        """
        Test loading a batch of data from disk.
        """
        datapoint = custom_dataset().load_datapoint(idx=1)
        assert datapoint["image"].shape == (150, 150)
        assert len(datapoint["metadata"].shape) == 1, "metadata should be a vector"
        assert datapoint["image"].dtype == torch.float32
        assert datapoint["metadata"].dtype == torch.float32

    def test_load_batch_rais_an_error_when_batch_idx_is_out_of_range(self):
        """
        Test that an error is raised when the batch index is out of range.
        """
        with self.assertRaises(IndexError):
            custom_dataset().load_datapoint(idx=custom_dataset().len)

    def test_load_image(self):
        dataset = custom_dataset()
        _dir = dataset.dirs[0]
        img = dataset.load_image(_dir)

        # check whether img is a PIL.Image object
        self.assertIsInstance(img, PIL.Image.Image)

        # assert that img dimensions are 150x150
        self.assertEqual(img.size, (150, 150))

    def test_load_metadata(self):
        dataset = custom_dataset()
        metadata = dataset.load_metadata(dataset.dirs[0])
        # assert that metadata is a dict
        self.assertTrue(isinstance(metadata, dict))

        # assert that metadata dict has  keys
        self.assertEqual(len(metadata.keys()), 2)

    def test_nav_model_forward(self):
        """
        Test the forward pass of a dummy model.
        """

        params = {
            "batch_size": 20,
            "num_workers": 0,
        }

        val_loader = DataLoader(custom_dataset(), **params)

        # init model
        model = NavModel()

        for img, meta, target in val_loader:
            # forward pass
            output = model(img, meta)

            # assert that output is a tensor
            self.assertTrue(isinstance(output, torch.Tensor))

            break  # only run once

    def test_mean_std(self):

        mean_std_dict = custom_dataset().calc_mean_std()

        # assert that mean_std_dict is a dict
        self.assertTrue(isinstance(mean_std_dict, dict))
        self.assertEqual(1, len(mean_std_dict["img_mean"]))
        self.assertEqual(1, len(mean_std_dict["img_std"]))
        self.assertEqual(55, len(mean_std_dict["meta_mean"]))
        self.assertEqual(55, len(mean_std_dict["meta_std"]))
        self.assertIsInstance(mean_std_dict["meta_max"], float)
        self.assertIsInstance(mean_std_dict["meta_min"], float)

    def test_init_transformers(self):
        val_set = custom_dataset()
        print(
            f"\nimg before normalization -> Max: {val_set[0][0].max():.2f}/ Min: {val_set[0][0].min():.2f}"
        )
        print(
            f"\nmeta before normalization -> Max: {val_set[0][1].max():.2f}/ Min: {val_set[0][1].min():.2f}"
        )
        val_set.init_transformers()
        print(
            f"\nimg after normalization -> Max: {val_set[0][0].max():.2f}/ Min: {val_set[0][0].min():.2f}"
        )
        print(
            f"\nmeta after normalization -> Max: {val_set[0][1].max():.2f}/ Min: {val_set[0][1].min():.2f}"
        )
