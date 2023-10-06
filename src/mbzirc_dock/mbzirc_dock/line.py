from __future__ import annotations

import numpy as np


class Line:
    def __init__(self, eq: np.ndarray, start: np.ndarray, end: np.ndarray) -> None:
        self.start = start
        self.end = end

    def length(self) -> float:
        return np.linalg.norm(self.end - self.start)

    def center(self) -> np.ndarray:
        return (self.start + self.end) / 2
