from typing import Optional, Tuple

import numpy as np
from rclpy.time import Time

from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException


class TargetPolicy(Policy):
    """Moves to circle around target using P controller assuming differential drive"""

    def __init__(self, v_orbit: float, r_orbit: float, precision: float, logger) -> None:
        super().__init__()
        self.log = logger
        self.v_orbit = v_orbit
        self.r_orbit = r_orbit
        self.precision = precision
        self.P_th = 0.2
        self.end = np.zeros(2)

    def get_action(self, prev_state: PolicyState) -> PolicyAction:
        self.end = self._get_end(prev_state)
        self.log.info(f'Calculated end: {self.end}')
        if abs(np.linalg.norm(prev_state.target) - self.r_orbit) <= self.precision:
            self.cancel()

        if self._is_canceled:
            raise PolicyCanceledException('Policy was canceled!')

        theta = np.arctan2(self.end[1], self.end[0])
        err_th = theta - 0.0
        om_z = self.P_th * err_th

        v_abs = self.v_orbit

        action = PolicyAction(
            linear_velocity=np.array([v_abs, 0, 0]),
            angular_velocity=np.array([0, 0, om_z])
        )

        return action

    def _get_end(self, state: PolicyState) -> np.ndarray:
        center = state.target
        radius = self.r_orbit
        vo = self.v_orbit

        # assume we are at (0, 0)
        d = np.linalg.norm(center)
        dr = np.sqrt(d**2 - radius**2)

        alpha = np.arctan2(center[1], center[0])
        beta = np.arcsin(radius / d)
        gamma = alpha + beta

        # r = PolicyState(
        #     time=self.end_time,
        #     position=np.array([dr * np.cos(gamma), dr * np.sin(gamma)]),
        #     velocity=np.array([vo * np.cos(gamma), vo * np.sin(gamma)]),
        #     acceleration=np.array([0, 0]),
        #     target=np.zeros(2)
        # )
        position = np.array([dr * np.cos(gamma), dr * np.sin(gamma)])
        return position
