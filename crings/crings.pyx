# distutils: language=c++

import numpy as np
cimport numpy as np
from libc.math cimport ceil as cceil

def fast_lidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs):
    rings = np.zeros(
        (scans.shape[0], angle_levels, range_levels, 1), dtype=np.uint8
    )
    clidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs, rings)
    return rings

# cdef clidar_to_rings():
def clidar_to_rings(
    np.float32_t[:, ::1] scans,
    int angle_levels,
    int range_levels,
    np.float32_t[::1] range_level_mins,
    np.float32_t[::1] range_level_maxs,
    np.uint8_t[:, :, :, ::1] rings,
):
    """
    scans: ndarray (n_scans, n_rays)   0-100 [m]
    rings: ndarray (n_scans, angle_levels, range_levels, n_channels)
    """
    cdef int n_scans = scans.shape[0]
    cdef int n_rays = scans.shape[1]
    cdef int CHANNEL = 0
    cdef np.float32_t downsample_factor = n_rays * 1.0 / angle_levels  # >= 1
    # vars
    cdef int ray_idx_start
    cdef int ray_idx_end
    cdef int angle_idx
    cdef int range_idx
    cdef int scan_idx
    cdef int ray_idx
    cdef np.float32_t r_min
    cdef np.float32_t r_max
    cdef np.float32_t dist
    cdef int cell_value
    for angle_idx in range(angle_levels):
        # generate ray indices corresponding to current angle level
        ray_idx_start = int(cceil(angle_idx * downsample_factor))
        ray_idx_end = int(cceil((angle_idx + 1) * downsample_factor))
        for range_idx in range(range_levels):
            r_min = range_level_mins[range_idx]
            r_max = range_level_maxs[range_idx]
            for scan_idx in range(n_scans):
                # initialize value of cell at (angle_idx, range_idx)
                cell_value = 0
                # check if any relevant rays hit cell or fall short
                for ray_idx in range(ray_idx_start, ray_idx_end):
                    dist = scans[scan_idx, ray_idx]
                    if dist == 0:
                        continue
                    # if a ray hits the cell the value is 2
                    if r_min <= dist and dist < r_max:
                        cell_value = 2
                        break
                    # if a ray falls short of the cell the value is at least 1
                    if dist < r_min:
                        cell_value = 1
                        continue
                rings[scan_idx, angle_idx, range_idx, CHANNEL] = cell_value
