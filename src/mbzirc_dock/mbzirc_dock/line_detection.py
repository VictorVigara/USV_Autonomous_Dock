from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

from mbzirc_dock.line import Line


def Pi(ph: np.ndarray) -> np.ndarray:
    return ph[:-1, :] / ph[-1, :]


def Pi_inv(pih: np.ndarray) -> np.ndarray:
    n_points = pih.shape[1]
    return np.vstack((pih, np.ones((1, n_points))))


def est_line(x: np.ndarray) -> np.ndarray:
    """Estimates line from points using PCA.
    :param x - 2xn nonhomogenous points"""
    d = np.cov(x)[:, 0]
    d /= np.linalg.norm(d)
    l = [d[1], -d[0]]
    l.append(-(l @ x.mean(1)))
    return np.array(l)


def separate(line: np.ndarray, points: np.ndarray, threshold: float) -> Tuple[np.ndarray, np.ndarray]:
    """Separates inliers and outliers on the line given threshold.
    :param line - 3x1 homogenous line
    :param points - 3xn homogenous points
    :param threshold - maximum distance from line to be counted as inlier

    :return separated inliers and outliers"""

    line_scaled = line / np.linalg.norm(line[:2])
    dists = np.abs(line_scaled.T @ points)
    inliers = points[:, dists <= threshold]
    outliers = points[:, dists > threshold]

    return inliers, outliers


def all_points_valid(line: np.ndarray, points: np.ndarray, threshold: float) -> int:
    """Checks if all points are inliers.
    :param line - 3x1 homogenous line
    :param points - 3xn homogenous points
    :param threshold - maximum distance from line to be counted as inlier

    :return number of inliers"""
    inliers, _ = separate(line, Pi_inv(points), threshold)
    return inliers.shape[1] == points.shape[1]


def detect_lines(points: np.ndarray, k: int, dist_threshold: float, log) -> List[Line]:
    """Detects continuous lines in ordered points set.
    :param points - 2xn ORDERED homogenous points
    :param k - number of consecutive points to estimate line
    :param dist_threshold - maximum distance from line to be counted as inlier

    :return List of Line obejcts"""
    n = points.shape[1]
    lines = []

    current_line: Optional[np.ndarray] = None

    i = 0
    while i < n - k + 1:
        j = k
        while i + j < n:
            iter_points = points[:, i:i + j]
            new_line = est_line(iter_points)

            if all_points_valid(new_line, iter_points, dist_threshold):
                current_line = new_line
                j += 1
            else:
                if current_line is not None:
                    start_ = points[:, i]
                    end_ = points[:, i + j - 2]
                    line = Line(current_line, start_, end_)
                    lines.append(line)
                i = i + j - 1
                current_line = None
                break

        # we exited loop without breaking, i.e. reached end of range, need to break
        if current_line is not None:
            start_ = points[:, i]
            end_ = points[:, i + j - 1]
            line = Line(current_line, start_, end_)
            lines.append(line)
            break
        if i + j >= n:
            break
    return lines
