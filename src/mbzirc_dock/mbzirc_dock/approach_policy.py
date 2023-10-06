from typing import Tuple

import numpy as np
from rclpy.time import Time

from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException


class ApproachPolicy(Policy):
    """Moves to the target using PID assuming differential drive"""

    def __init__(self, target: np.ndarray, v: float, precision: float, logger) -> None:
        super().__init__()
        self.log = logger
        self.target = target
        self.v = v
        self.precision = precision
        self.P_th = 0.5

    def get_action(self, prev_state: PolicyState) -> PolicyAction:
        if np.linalg.norm(self.target) <= self.precision:
            self.log.info(f'[ApproachPolicy] Precision reached: {np.linalg.norm(self.target)}')
            self.cancel()

        if self._is_canceled:
            raise PolicyCanceledException('Policy was canceled!')

        theta = np.arctan2(self.target[1, 0], self.target[0, 0])
        err_th = theta - 0.0
        om_z = self.P_th * err_th
        self.log.info(f'om_z: {om_z} (rad/s)')

        # slow down if the angle is too big
        if np.abs(err_th) > np.deg2rad(20):
            v_abs = self.v / 4
        else:
            v_abs = self.v

        action = PolicyAction(
            linear_velocity=np.array([v_abs, 0, 0]),
            angular_velocity=np.array([0, 0, om_z])
        )

        return action
