import math

import numpy as np


class Transform:
    def __init__(self, scale, rotation, translation):
        self._scale = float(scale)
        self._rotation = float(rotation)
        self._translation = np.asarray(translation, dtype=float)

    def get_rotation(self):
        return self._rotation

    def get_scale(self):
        return self._scale

    def get_translation(self):
        return self._translation.tolist()

    def transform(self, point):
        point = np.asarray(point, dtype=float)
        c = math.cos(self._rotation)
        s = math.sin(self._rotation)
        rot = np.array([[c, -s], [s, c]], dtype=float)
        result = self._scale * (rot @ point) + self._translation
        return result.tolist()


def estimate(src_points, dst_points):
    src = np.asarray(src_points, dtype=float)
    dst = np.asarray(dst_points, dtype=float)

    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2:
        raise ValueError("estimate expects Nx2 source and destination points")

    n = src.shape[0]
    if n == 0:
        return Transform(1.0, 0.0, [0.0, 0.0])

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    cov = (src_centered.T @ dst_centered) / n
    u, d, vt = np.linalg.svd(cov)

    s_mat = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_mat[-1, -1] = -1

    rot_mat = u @ s_mat @ vt
    var_src = np.mean(np.sum(src_centered ** 2, axis=1))
    if var_src <= 1e-12:
        scale = 1.0
    else:
        scale = np.trace(np.diag(d) @ s_mat) / var_src

    translation = dst_mean - scale * (rot_mat @ src_mean)
    rotation = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    return Transform(scale, rotation, translation)


def estimate_error(transform, src_points, dst_points):
    src = np.asarray(src_points, dtype=float)
    dst = np.asarray(dst_points, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2:
        raise ValueError("estimate_error expects Nx2 source and destination points")

    transformed = np.asarray([transform.transform(p) for p in src], dtype=float)
    errors = np.sum((transformed - dst) ** 2, axis=1)
    return float(np.mean(errors))
