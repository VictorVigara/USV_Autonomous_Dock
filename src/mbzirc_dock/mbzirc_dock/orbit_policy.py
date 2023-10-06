from typing import Tuple

import numpy as np
from rclpy.time import Time

from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException


State2D = Tuple[np.ndarray, np.ndarray, np.ndarray]


def time_to_float(time: Time) -> float:
    return time.nanoseconds / 1e9


class OrbitPolicy(Policy):
    """Orbits target point at constant speed and uses P control to adapt angular velocity."""
    def __init__(self, radius: float, speed: float, logger) -> None:
        super().__init__()
        self.log = logger
        self._r = radius
        self._v = speed

        self.theta_P = 0.1
        self.om_P = 0.6

    def get_action(self, state: PolicyState) -> PolicyAction:
        if self._is_canceled:
            raise PolicyCanceledException('Policy was canceled!')

        alpha = np.arctan2(state.target[1], state.target[0])
        d = np.linalg.norm(state.target)
        dist_err = self._r - d
        self.log.info(f'Distance error: {dist_err:.2f}')

        ANGLE_THRES = np.deg2rad(30)
        # if we are far (dist_err <  0), we want angle to be more than -90deg
        theta_offset = self.theta_P * dist_err
        theta_offset = np.clip(theta_offset, -ANGLE_THRES, ANGLE_THRES)
        target_theta = -np.pi / 2 - theta_offset
        self.log.info(f'Target angle: {np.rad2deg(target_theta):.2f} deg')
        angle_err = target_theta - alpha
        self.log.info(f' Angle error: {np.rad2deg(angle_err):.2f} deg')

        # if we have positive error, then we need to rotate in opposite direction
        om_z = -angle_err * self.om_P

        action = PolicyAction(
            linear_velocity=np.array([self._v, 0, 0]),
            angular_velocity=np.array([0, 0, om_z])
        )

        return action
