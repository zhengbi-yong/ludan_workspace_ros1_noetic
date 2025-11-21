#!/usr/bin/env python3
"""Action filtering module for smoothing and damping actions."""

from typing import Optional

import numpy as np


class ActionFilter:
    """Filters actions using exponential moving average and damping."""

    def __init__(self, filter_alpha: float = 0.25, damping: float = 0.0):
        """
        Initialize action filter.

        Args:
            filter_alpha: Exponential moving average coefficient (0-1)
            damping: Damping coefficient (0-1), reduces action changes
        """
        self.filter_alpha = max(0.0, min(1.0, filter_alpha))
        self.damping = max(0.0, min(1.0, damping))
        self.filtered_action: Optional[np.ndarray] = None

    def filter(self, action: np.ndarray, previous_action: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Filter action using exponential moving average and damping.

        Args:
            action: Raw action array
            previous_action: Previous action for damping (optional)

        Returns:
            Filtered action array
        """
        if self.filtered_action is None:
            self.filtered_action = action.copy()
            return self.filtered_action

        # Apply damping if enabled
        if self.damping > 0.0 and previous_action is not None:
            delta = action - self.filtered_action
            action = self.filtered_action + (1.0 - self.damping) * delta

        # Apply exponential moving average
        self.filtered_action = self.filtered_action + self.filter_alpha * (
            action - self.filtered_action
        )

        return self.filtered_action

    def reset(self):
        """Reset filter state."""
        self.filtered_action = None

