import numpy as np


class Tracker2D:
    def __init__(self, x0: np.ndarray, P0: float, R0: float, Q0: float, dt: float, mode: str) -> None:
        self.x = np.array([[
            x0[0],  # x position
            0.0,    # x velocity
            0.0,    # x acceleration
            x0[1],  # y position
            0.0,    # y velocity
            0.0     # y acceleration
        ]]).T

        self.P = P0 * np.eye(6)     # state uncertainty
        self.R = R0 * np.eye(2)     # measurement uncertainty
        self.Q = Q0 * np.eye(6)     # process noise

        self.H = np.array([  # observation matrix
            [1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0]
        ])

        if mode == 'p':     # only position tracking
            self.F = np.array([  # transition matrix
                [1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
            ])
        elif mode == 'v':     # position & velocity tracking
            self.F = np.array([  # transition matrix
                [1, dt, 0, 0,  0, 0],
                [0,  1, 0, 0,  0, 0],
                [0,  0, 0, 0,  0, 0],
                [0,  0, 0, 1, dt, 0],
                [0,  0, 0, 0,  1, 0],
                [0,  0, 0, 0,  0, 0],
            ])
        elif mode == 'a':     # position, velocity & acceleration tracking
            a = dt ** 2 / 2  # acceleration term
            self.F = np.array([  # transition matrix
                [1, dt,  a, 0,  0,  0],
                [0,  1, dt, 0,  0,  0],
                [0,  0,  1, 0,  0,  0],
                [0,  0,  0, 1, dt,  a],
                [0,  0,  0, 0,  1, dt],
                [0,  0,  0, 0,  0,  1],
            ])
        else:
            raise NotImplementedError(f'Invalid mode: {mode}, only p, v, a available.')

    def update(self, Z: np.ndarray) -> None:
        """Update state with new measurement."""
        err = Z - self.H @ self.x
        print(f'{self.H.shape}')
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ err
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def predict(self) -> None:
        """Predict next state."""
        self.x = self.F @ self.x  # + u
        self.P = self.F @ self.P @ self.F.T + self.Q

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x[0, 0], self.x[3, 0]])
