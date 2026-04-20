import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import gymnasium as gym
from gymnasium import spaces

from airgym.wrappers.curriculum import CurriculumWrapper
from config import DEFAULT_CONFIG, OBSERVATION_SCHEMA_VERSION, load_config, validate_config


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
    def test_default_config_enables_curriculum_by_default(self):
        config = load_config(DEFAULT_CONFIG)

        self.assertTrue(config.curriculum.enabled)

    def test_default_config_uses_split_crossq_architecture_on_cuda(self):
        config = load_config(DEFAULT_CONFIG)

        self.assertEqual(config.algo.device, "cuda")
        self.assertEqual(list(config.algo.net_arch.pi), [256, 256])
        self.assertEqual(list(config.algo.net_arch.qf), [2048, 2048])

    def test_default_config_starts_with_warmup_stage(self):
        config = load_config(DEFAULT_CONFIG)
        first_stage = config.curriculum.stages[0]

        self.assertEqual(first_stage.name, "stage_0_warmup")
        self.assertEqual(first_stage.num_obstacles, 0)
        self.assertEqual(first_stage.num_dynamic_obstacles, 0)
        self.assertEqual(first_stage.num_waypoints, 1)
        self.assertEqual(first_stage.length, 4)
        self.assertEqual(list(first_stage.angle_range), [-10.0, 10.0])

    def test_child_config_in_another_directory_still_loads_warmup_stage(self):
        with TemporaryDirectory() as temp_dir:
            child_dir = Path(temp_dir) / "child_configs"
            child_dir.mkdir()
            child_config = child_dir / "child.yaml"
            relative_base = Path(os.path.relpath(DEFAULT_CONFIG.with_name("base.yaml"), child_dir))
            child_config.write_text(
                f"extends: {relative_base.as_posix()}\n"
                "train:\n"
                "  total_timesteps: 123\n",
                encoding="utf-8",
            )

            config = load_config(child_config)

        self.assertTrue(config.curriculum.enabled)
        self.assertGreater(len(config.curriculum.stages), 0)
        self.assertEqual(config.curriculum.stages[0].name, "stage_0_warmup")

    def test_child_local_curriculum_file_does_not_override_base_declared_file(self):
        with TemporaryDirectory() as temp_dir:
            child_dir = Path(temp_dir) / "child_configs"
            child_dir.mkdir()
            child_config = child_dir / "child.yaml"
            child_curriculum = child_dir / "curriculum.yaml"
            relative_base = Path(os.path.relpath(DEFAULT_CONFIG.with_name("base.yaml"), child_dir))
            child_config.write_text(f"extends: {relative_base.as_posix()}\n", encoding="utf-8")
            child_curriculum.write_text(
                "stages:\n"
                "  - name: child_local_stage\n"
                "    num_obstacles: 99\n"
                "    num_dynamic_obstacles: 0\n"
                "    num_waypoints: 1\n"
                "    length: 1\n"
                "    angle_range: [0.0, 0.0]\n",
                encoding="utf-8",
            )

            config = load_config(child_config)

        self.assertGreater(len(config.curriculum.stages), 0)
        self.assertEqual(config.curriculum.stages[0].name, "stage_0_warmup")
        self.assertNotEqual(config.curriculum.stages[0].name, "child_local_stage")

    def test_curriculum_file_override_loads_custom_stage_file(self):
        with TemporaryDirectory() as temp_dir:
            custom_curriculum = Path(temp_dir) / "custom_curriculum.yaml"
            custom_curriculum.write_text(
                "stages:\n"
                "  - name: custom_override_stage\n"
                "    num_obstacles: 7\n"
                "    num_dynamic_obstacles: 0\n"
                "    num_waypoints: 2\n"
                "    length: 9\n"
                "    angle_range: [-15.0, 15.0]\n",
                encoding="utf-8",
            )

            config = load_config(
                DEFAULT_CONFIG,
                [f"curriculum.file={custom_curriculum.as_posix()}"],
            )

        self.assertEqual(len(config.curriculum.stages), 1)
        self.assertEqual(config.curriculum.stages[0].name, "custom_override_stage")
        self.assertEqual(config.curriculum.stages[0].num_obstacles, 7)

    def test_relative_curriculum_file_override_resolves_from_leaf_config_directory(self):
        with TemporaryDirectory() as temp_dir:
            child_dir = Path(temp_dir) / "child_configs"
            child_dir.mkdir()
            child_config = child_dir / "child.yaml"
            child_curriculum = child_dir / "relative_curriculum.yaml"
            relative_base = Path(os.path.relpath(DEFAULT_CONFIG.with_name("base.yaml"), child_dir))
            child_config.write_text(f"extends: {relative_base.as_posix()}\n", encoding="utf-8")
            child_curriculum.write_text(
                "stages:\n"
                "  - name: child_relative_override_stage\n"
                "    num_obstacles: 5\n"
                "    num_dynamic_obstacles: 0\n"
                "    num_waypoints: 2\n"
                "    length: 8\n"
                "    angle_range: [-12.0, 12.0]\n",
                encoding="utf-8",
            )

            config = load_config(
                child_config,
                ["curriculum.file=relative_curriculum.yaml"],
            )

        self.assertEqual(len(config.curriculum.stages), 1)
        self.assertEqual(config.curriculum.stages[0].name, "child_relative_override_stage")
        self.assertEqual(config.curriculum.stages[0].num_obstacles, 5)

    def test_relative_curriculum_file_override_does_not_fall_back_to_base_directory(self):
        with TemporaryDirectory() as temp_dir:
            child_dir = Path(temp_dir) / "child_configs"
            child_dir.mkdir()
            child_config = child_dir / "child.yaml"
            relative_base = Path(os.path.relpath(DEFAULT_CONFIG.with_name("base.yaml"), child_dir))
            child_config.write_text(f"extends: {relative_base.as_posix()}\n", encoding="utf-8")

            with self.assertRaisesRegex(FileNotFoundError, r"curriculum\.yaml"):
                load_config(
                    child_config,
                    ["curriculum.file=curriculum.yaml"],
                )

    def test_validate_config_rejects_enabled_curriculum_without_stages(self):
        config = load_config(DEFAULT_CONFIG)
        config.curriculum.stages = []

        with self.assertRaisesRegex(ValueError, "curriculum.enabled.*zero stages|no stages|at least one stage"):
            validate_config(config)

    def test_validate_config_rejects_mismatched_observation_schema_version(self):
        config = load_config(DEFAULT_CONFIG)
        config.env.observation_schema_version = OBSERVATION_SCHEMA_VERSION - 1

        with self.assertRaisesRegex(
            ValueError,
            rf"observation_schema_version={OBSERVATION_SCHEMA_VERSION - 1}.*expected {OBSERVATION_SCHEMA_VERSION}",
        ):
            validate_config(config)

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
