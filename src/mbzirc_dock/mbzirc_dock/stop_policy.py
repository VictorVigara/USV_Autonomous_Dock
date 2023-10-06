from typing import Tuple

import numpy as np
from rclpy.time import Time

from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException


class StopPolicy(Policy):
    def __init__(self) -> None:
        super().__init__()
        self.prev_state = None

    def get_action(self, prev_state: PolicyState) -> PolicyAction:
        if self._is_canceled:
            raise PolicyCanceledException('Policy was canceled!')
        self.prev_state = prev_state

        action = PolicyAction(np.zeros(3), np.zeros(3))

        return action
