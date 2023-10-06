from typing import Optional, Tuple

import numpy as np
from rclpy.time import Time

from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException
from mbzirc_dock.tracker import Tracker2D


class TouchPolicy(Policy):
    """Moves to the target using P controller assuming omnidrive"""

    def __init__(self, line_low: Tracker2D, line_high: Tracker2D,
                 usv_width: float, v: float, precision: float,
                 callback: callable, logger) -> None:
        super().__init__()
        self.log = logger
        self.line_low = line_low
        self.line_high = line_high
        self.usv_width = usv_width
        self.v = v
        self.precision = precision
        self.P_x = 0.2
        self.P_th = 0.8

        self.callback = callback

    def get_action(self, prev_state: PolicyState) -> PolicyAction:
        # get line ends relative to the center of the side of the USV
        line_low = self.line_low.pos + np.array([0, self.usv_width / 2])
        line_high = self.line_high.pos + np.array([0, self.usv_width / 2])

        self.log.info(f'Line low: {line_low}')
        self.log.info(f'Line high: {line_high}')
        dist = max(line_low[1], line_high[1])
        if -dist <= self.precision:
            self.log.info(f'[TouchPolicy] Precision reached: {dist}')
            self.callback()
            self.cancel()

        if self._is_canceled:
            raise PolicyCanceledException('Policy was canceled!')

        err_x = line_high[0] - line_low[0]
        v_x  = self.P_x * err_x
        v_y = -self.v
        v_x, v_y = self.v * np.array([v_x, v_y]) / np.linalg.norm([v_x, v_y])

        err_y = line_high[1] - line_low[1]
        om_z = self.P_th * err_y

        action = PolicyAction(
            linear_velocity=np.array([v_x, v_y, 0]),
            angular_velocity=np.array([0, 0, om_z])
        )

        return action
