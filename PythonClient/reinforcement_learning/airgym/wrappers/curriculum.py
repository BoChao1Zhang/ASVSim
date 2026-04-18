from __future__ import annotations

from collections import deque

import gymnasium as gym
from diagnostics import diag_log


class CurriculumWrapper(gym.Wrapper):
    def __init__(self, env, curriculum_config, base_seed: int = 0):
        super().__init__(env)
        self.curriculum_config = curriculum_config
        self.base_seed = int(base_seed)
        self.stage_index = 0
        self._episode_counter = 0
        self._history = deque(maxlen=int(curriculum_config.promote_window))

        if not curriculum_config.stages:
            raise ValueError("CurriculumWrapper requires at least one curriculum stage.")
        self._schedule_next_episode()

    @property
    def current_stage(self):
        return self.curriculum_config.stages[self.stage_index]

    def _schedule_next_episode(self):
        stage = self.current_stage
        episode_seed = self.base_seed + self.stage_index * 100_000 + self._episode_counter
        diag_log(
            "curriculum_schedule_next_episode",
            stage_index=self.stage_index,
            stage_name=stage.name,
            episode_counter=self._episode_counter,
            episode_seed=episode_seed,
            num_obstacles=int(stage.num_obstacles),
            num_waypoints=int(stage.num_waypoints),
            length=int(stage.length),
        )
        self.env.set_next_episode_params(
            num_obstacles=int(stage.num_obstacles),
            num_dynamic_obstacles=int(stage.num_dynamic_obstacles),
            goal_distance=int(stage.num_waypoints),
            terrain_length=int(stage.length),
            angle_range=[float(stage.angle_range[0]), float(stage.angle_range[1])],
            terrain_seed=episode_seed,
            force_terrain_regen=True,
        )

    def reset(self, **kwargs):
        self._schedule_next_episode()
        observation, info = self.env.reset(**kwargs)
        info["curriculum_stage"] = self.stage_index
        info["curriculum_stage_name"] = self.current_stage.name
        diag_log("curriculum_reset", stage_index=self.stage_index, stage_name=self.current_stage.name)
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = self.env.step(action)
        episode_stage_index = self.stage_index
        episode_stage_name = self.current_stage.name
        info["curriculum_stage"] = episode_stage_index
        info["curriculum_stage_name"] = episode_stage_name

        if terminated or truncated:
            success = info.get("end_reason") == "goal_reached"
            self._history.append(1 if success else 0)
            self._episode_counter += 1

            promote = (
                len(self._history) == self._history.maxlen
                and (sum(self._history) / len(self._history)) >= float(self.curriculum_config.promote_threshold)
                and self.stage_index < len(self.curriculum_config.stages) - 1
            )
            if promote:
                self.stage_index += 1
                self._history.clear()
                info["curriculum_promoted"] = 1
            else:
                info["curriculum_promoted"] = 0

            info["episode_stage_before_promotion"] = episode_stage_index
            info["episode_stage_name_before_promotion"] = episode_stage_name
            info["episode_stage_after_promotion"] = self.stage_index
            info["episode_stage_name_after_promotion"] = self.current_stage.name
            diag_log(
                "curriculum_episode_complete",
                success=success,
                promoted=bool(info["curriculum_promoted"]),
                stage_before=episode_stage_index,
                stage_name_before=episode_stage_name,
                stage_after=self.stage_index,
                stage_name_after=self.current_stage.name,
                history=list(self._history),
            )
            self._schedule_next_episode()

        return observation, reward, terminated, truncated, info
