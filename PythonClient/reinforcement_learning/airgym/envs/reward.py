from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np


def _cfg_get(config, key: str, default):
    if isinstance(config, Mapping):
        return config.get(key, default)
    return getattr(config, key, default)


class RewardComputer:
    def __init__(self, reward_config):
        self.reward_config = reward_config

    def compute(self, state, prev_state, action, prev_action):
        dt = float(state.get("dt", 0.25))
        step_scale = dt / 0.25
        current_distance = float(state.get("distance_to_goal", 0.0))
        previous_distance = float(prev_state.get("distance_to_goal", current_distance))
        progress = previous_distance - current_distance

        heading_error = float(state.get("heading_error", 0.0))
        heading_align = math.cos(heading_error)

        min_obstacle_distance = float(state.get("min_obstacle_distance", 999.0))
        obstacle_decay = float(_cfg_get(self.reward_config, "obstacle_decay", 5.0))
        obstacle_proximity = -math.exp(-min_obstacle_distance / max(obstacle_decay, 1e-6))

        action_delta = np.asarray(action, dtype=np.float32) - np.asarray(prev_action, dtype=np.float32)
        action_rate = -float(np.dot(action_delta, action_delta))

        cross_track = -abs(float(state.get("cross_track_error", 0.0)))
        step_penalty = -1.0 * step_scale
        forward_velocity = max(float(state.get("v_los", 0.0)), 0.0) * step_scale

        speed = float(state.get("speed", 0.0))
        elapsed_time = float(state.get("elapsed_time", 0.0))
        stall_speed_threshold = float(_cfg_get(self.reward_config, "stall_speed_threshold", 0.3))
        stall_warmup_seconds = float(_cfg_get(self.reward_config, "stall_warmup_seconds", 10.0))
        stall_penalty = 0.0
        if speed < stall_speed_threshold and elapsed_time > stall_warmup_seconds:
            stall_penalty = -1.0 * step_scale

        terminal_raw = 0.0
        if state.get("collision", False):
            terminal_raw = float(_cfg_get(self.reward_config, "terminal_collision", -100.0))
        elif state.get("goal_reached", False):
            terminal_raw = float(_cfg_get(self.reward_config, "terminal_goal", 500.0))

        components = {
            "progress": float(_cfg_get(self.reward_config, "progress", 1.0)) * progress,
            "heading_align": float(_cfg_get(self.reward_config, "heading_align", 0.0)) * heading_align,
            "obstacle_proximity": float(_cfg_get(self.reward_config, "obstacle_proximity", 0.0)) * obstacle_proximity,
            "action_rate": float(_cfg_get(self.reward_config, "action_rate", 0.0)) * action_rate,
            "cross_track": float(_cfg_get(self.reward_config, "cross_track", 0.0)) * cross_track,
            "step_penalty": float(_cfg_get(self.reward_config, "step_penalty", 0.0)) * step_penalty,
            "forward_velocity": float(_cfg_get(self.reward_config, "forward_velocity", 0.0)) * forward_velocity,
            "stall_penalty": float(_cfg_get(self.reward_config, "stall_penalty", 0.0)) * stall_penalty,
            "terminal": float(_cfg_get(self.reward_config, "terminal", 1.0)) * terminal_raw,
        }
        total_reward = float(sum(components.values()))
        return total_reward, components
