from batchgenerators.transforms.color_transforms import ContrastAugmentationTransform
from batchgenerators.transforms.spatial_transforms import MirrorTransform
from batchgenerators.transforms.abstract_transforms import Compose
from batchgenerators.transforms.spatial_transforms import SpatialTransform
from batchgenerators.dataloading.multi_threaded_augmenter import MultiThreadedAugmenter
from batchgenerators.transforms.color_transforms import ContrastAugmentationTransform
from batchgenerators.dataloading.data_loader import DataLoaderBase
import numpy as np
from skimage import data


from utils import plot_batch

class DataLoader(DataLoaderBase):
    def __init__(self, data, BATCH_SIZE=2, num_batches=None, seed=False):
        super().__init__(data, BATCH_SIZE, num_batches, seed)
    
    def generate_train_batch(self):
        img = self._data  # img: numpy.ndarray

        img = np.tile(img[None, None], (self.BATCH_SIZE, 1, 1, 1))
        return {'data':img.astype(np.float32), 'some_other_key':'some other value'}

batchgen = DataLoader(data.camera(), 4, None, False)
batch = next(batchgen)

def transform():
    my_transforms = []
    brightness_transform = ContrastAugmentationTransform((0.3, 3.), preserve_range=True)
    my_transforms.append(brightness_transform)
    mirror_transform = MirrorTransform(axes=(0, 1))
    my_transforms.append(mirror_transform)
    spatial_transform = SpatialTransform(data.camera().shape, np.array(data.camera().shape) // 2, 
                 do_elastic_deform=True, alpha=(0., 1500.), sigma=(30., 50.),
                 do_rotation=True, angle_z=(0, 2 * np.pi),
                 do_scale=True, scale=(0.3, 3.), 
                 border_mode_data='constant', border_cval_data=0, order_data=1,
                 random_crop=False)
    my_transforms.append(spatial_transform)
    all_transforms = Compose(my_transforms)
    multithreaded_generator = MultiThreadedAugmenter(batchgen, all_transforms, 4, 2, seeds=None)
    
    return multithreaded_generator

        
if __name__ == "__main__":
    #from multiprocessing import freeze_support
    #freeze_support()
    plot_batch(transform().next())