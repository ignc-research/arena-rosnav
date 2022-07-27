# load maps

from batchgenerators.dataloading.data_loader import DataLoaderBase
import numpy as np

class mapLoader(DataLoaderBase):
    def __init__(self, data, BATCH_SIZE=5, num_batches=None, seed=False):
        super().__init__(data, BATCH_SIZE, num_batches, seed)
        print(data)
        
    def generate_train_batch(self):
        img = self._data
        return {'data': img}
        