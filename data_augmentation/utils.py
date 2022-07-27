import matplotlib.pyplot as plt
from pathlib import Path
from PIL import Image
import numpy as np

currentPath = Path(__file__)

def plot_batch(batch):
    batch_size = batch['data'].shape[0]
    plt.figure(figsize=(16, 10))
    for i in range(batch_size):
        plt.subplot(1, batch_size, i+1)
        plt.imshow(batch['data'][i, 0], cmap="gray") # only grayscale image here
    plt.show()
    

def load_data_from_path(path: Path=currentPath.parents[1]):
    imgs = []
    path = path / "dataset" / "maps"
    for map in path.iterdir():
        for map in map.glob('*.png'):
            img = Image.open(map)
            img.convert('RGBA')
            img = np.asarray(img)
            imgs.append(img)
    return imgs
                    

load_data_from_path()