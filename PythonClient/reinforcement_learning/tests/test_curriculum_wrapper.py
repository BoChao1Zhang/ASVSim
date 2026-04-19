from pathlib import Path
from types import SimpleNamespace
import unittest

import gymnasium as gym
from gymnasium import spaces

from airgym.wrappers.curriculum import CurriculumWrapper
from config import load_config


BASE_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "base.yaml"


class _FakeEnv(gym.Env):
    metadata = {}

    def __init__(self):
        super().__init__()
        self.action_space = spaces.Discrete(1)
        self.observation_space = spaces.Discrete(1)
        self.scheduled_params = []

    def set_next_episode_params(self, **kwargs):
        self.scheduled_params.append(kwargs)

    def reset(self, **kwargs):
        return 0, {}

    def step(self, action):
        return 0, 1.0, True, False, {"end_reason": "goal_reached"}


class CurriculumWrapperTests(unittest.TestCase):
    def test_base_config_enables_curriculum_by_default(self):
        config = load_config(BASE_CONFIG)

        self.assertTrue(config.curriculum.enabled)

    def test_base_config_starts_with_warmup_stage(self):
        config = load_config(BASE_CONFIG)
        first_stage = config.curriculum.stages[0]

        self.assertEqual(first_stage.name, "stage_0_warmup")
        self.assertEqual(first_stage.num_obstacles, 0)
        self.assertEqual(first_stage.num_dynamic_obstacles, 0)
        self.assertEqual(first_stage.num_waypoints, 1)
        self.assertEqual(first_stage.length, 4)
        self.assertEqual(list(first_stage.angle_range), [-10.0, 10.0])

    def test_terminal_episode_preserves_stage_before_promotion(self):
        curriculum_config = SimpleNamespace(
            promote_window=1,
            promote_threshold=1.0,
            stages=[
                SimpleNamespace(
                    name="stage_0",
                    num_obstacles=1,
                    num_dynamic_obstacles=0,
                    num_waypoints=1,
                    length=10,
                    angle_range=[-45.0, 45.0],
                ),
                SimpleNamespace(
                    name="stage_1",
                    num_obstacles=2,
                    num_dynamic_obstacles=0,
                    num_waypoints=2,
                    length=20,
                    angle_range=[-30.0, 30.0],
                ),
            ],
        )
        env = _FakeEnv()
        wrapper = CurriculumWrapper(env, curriculum_config, base_seed=11)

        wrapper.reset()
        _, _, terminated, truncated, info = wrapper.step(0)

        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["curriculum_promoted"], 1)
        self.assertEqual(info["curriculum_stage"], 0)
        self.assertEqual(info["curriculum_stage_name"], "stage_0")
        self.assertEqual(info["episode_stage_before_promotion"], 0)
        self.assertEqual(info["episode_stage_after_promotion"], 1)
        self.assertEqual(wrapper.stage_index, 1)
        self.assertEqual(env.scheduled_params[-1]["num_obstacles"], 2)


if __name__ == "__main__":
    unittest.main()
