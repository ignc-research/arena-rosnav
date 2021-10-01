import numpy as np
from matplotlib import pyplot as plt

from crings import fast_lidar_to_rings
g = 0

# Modify based on the ring.py.bak for pickle the function
# for testing and visalization and experiments, you should still use that file

def lidar_to_rings(scans,angle_levels, range_levels, range_level_mins, range_level_maxs):
    """
    scans: ndarray (n, N_RAYS)   0-100 [m]
    rings: ndarray (n_scans, angle_levels, range_levels, n_channels)
    """
    return fast_lidar_to_rings(scans,angle_levels, range_levels, range_level_mins, range_level_maxs)


def generate_rings(
    angle_levels=64,
    range_levels=64,
    expansion_term=3.12,
    min_resolution=0.01,
    min_dist=0.3,
    max_dist=25.0,
    VISUALIZE=False,
):
    """
    expansion_term: 0. (equal range levels) 1. (linear increase in depths), >1. (exponential)

    range levels
    ---
    first bin is           0          to   min_dist
    second bin is          min_dist   to   min_resolution
    nth bin is             x          to   x
    ...
    second-to-last bin is  x          to   max_dist
    last bin is            max_dist   to   inf

    bin values
    ---
    0 points in bin     0
    1 points in bin     0.5
    2 points in bin     0.75
    ..
    inf points in bin   1.
    (a.k.a x(n) = 1-2^-n)
    """
    x = np.linspace(0, 1, range_levels - 2)
    expansion_curve = np.power(x, expansion_term)  # a.k.a range_level_depths
    renormalisation_factor = np.sum(expansion_curve) / (
        max_dist - (min_resolution * (range_levels - 2)) - min_dist
    )
    range_level_depths = expansion_curve / renormalisation_factor + min_resolution
    range_level_maxs = np.cumsum(range_level_depths) + min_dist
    range_level_maxs = np.concatenate([[min_dist], range_level_maxs, [np.inf]]).astype(np.float32)
    range_level_mins = np.concatenate([[0.0], range_level_maxs[:-1]]).astype(np.float32)

    if VISUALIZE:
        th = np.linspace(-7.0 / 8.0 * np.pi, 7.0 / 8.0 * np.pi, angle_levels)
        plt.figure("curve")
        plt.plot(range_level_maxs[:-1])
        for x, y in enumerate(range_level_maxs[:-1]):
            plt.axhline(y)
            plt.axvline(x)
        plt.figure("rings")
        for i, r in enumerate(range_level_maxs[:-1]):
            plt.gca().add_artist(plt.Circle((0, 0), r, color="k", fill=False))
        for i in range(angle_levels):
            plt.plot(
                [min_dist * np.cos(th), r * np.cos(th)],
                [min_dist * np.sin(th), r * np.sin(th)],
                "k",
            )
            plt.axis("equal")
        plt.gca().add_artist(plt.Circle((1, 1), 0.3, color="r", zorder=3))
        plt.tight_layout()
        plt.savefig("test_data.png")




    return {
        "range_level_mins": range_level_mins,
        "range_level_maxs": range_level_maxs,
        "lidar_to_rings": lidar_to_rings,
        "rings_to_lidar": rings_to_lidar,
        "visualize_rings": visualize_rings,
        "rings_to_bool": 2.0,
    }


def generate_downsampling_map(I, J):
    """
    with,
    I = 5
    J = 2

    0   1               I
    |___|___|___|___|___|
    |_________|_________|
    0         1         J

    >> i_to_j, j_to_ii = downsample_map(5, 2)
    >> i_to_j
    [0, 0, 0, 1, 1]
    >> j_to_ii[0]
    [0, 1, 2]
    >> j_to_ii[1]
    [3, 4]
    """
    downsample_factor = I * 1.0 / J
    i_to_j = np.floor(np.arange(I) / downsample_factor).astype(int)
    j_to_ii = [
        np.arange(
            np.ceil(j * downsample_factor),
            np.ceil((j + 1) * downsample_factor),
            dtype=int,
        )
        for j in range(J)
    ]
    return i_to_j, j_to_ii


if __name__ == "__main__":
    import time
    ring_def = generate_rings(expansion_term=0,min_dist=0.1,max_dist=3.5,VISUALIZE = True)
    # linear ramp
    scans = np.ones((1, 360)) * (np.arange(360) / 360.0 * 3.5)[None, :]
    theta = 2*np.pi/360*np.arange(360)
    x = scans*np.sin(theta)
    y = scans*np.cos(theta)
    plt.figure()
    plt.plot(x,y)
    print(x,y)
    plt.savefig('test_original.png')
    t_start = time.time()
    rings = ring_def["lidar_to_rings"](scans.astype(np.float32))
    t_end = time.time()
    print(t_end-t_start)
    plt.ion()
    ring_def["visualize_rings"](rings[0, :, :, :], scan=scans[0])


    ring_def = generate_rings()
    # linear ramp
    scans = np.ones((1, 1080)) * (np.arange(1080) / 1080.0 * 25.0)[None, :]
    rings = ring_def["lidar_to_rings"](scans.astype(np.float32))
    plt.ion()
    ring_def["visualize_rings"](rings[0, :, :, :], scan=scans[0])
