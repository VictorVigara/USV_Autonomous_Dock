from __future__ import annotations

import numpy as np
    
class Line:
    def __init__(self, eq: np.ndarray, start: np.ndarray, end: np.ndarray, pts) -> None:
        self.start = start
        self.end = end
        self.eq = eq
        self.pts = pts

    def length(self) -> float:
        return np.linalg.norm(self.end - self.start)

    def center(self) -> np.ndarray:
        return (self.start + self.end) / 2
    
    def line_eq(self): 
        return self.eq
    
    def points(self): 
        return self.pts
    
    def n_points(self): 
        return self.pts.shape[1]
