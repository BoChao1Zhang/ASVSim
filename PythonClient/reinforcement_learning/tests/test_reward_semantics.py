import unittest
from types import SimpleNamespace
from unittest import mock

from airgym.envs.reward import RewardComputer
from airgym.envs.vessel_env import PCGVesselEnv
from config import DEFAULT_CONFIG, load_config, validate_config


class RewardSemanticsTests(unittest.TestCase):
    def test_validate_config_rejects_negative_reward_shaping_fields(self):
        config = load_config(DEFAULT_CONFIG)
        config.reward.forward_velocity = -0.1

        with self.assertRaisesRegex(ValueError, "reward.forward_velocity"):
            validate_config(config)

    def test_step_penalty_scales_with_dt(self):
        reward_computer = RewardComputer(
            {
                "progress": 0.0,
                "heading_align": 0.0,
                "obstacle_proximity": 0.0,
                "action_rate": 0.0,
                "cross_track": 0.0,
                "step_penalty": 0.1,
                "terminal": 1.0,
            }
        )

        reward, components = reward_computer.compute(
            state={"dt": 0.5},
            prev_state={},
            action=[0.0, 0.0],
            prev_action=[0.0, 0.0],
        )

        self.assertAlmostEqual(components["step_penalty"], -0.2)
        self.assertAlmostEqual(reward, -0.2)

    def test_forward_velocity_reward_is_positive_when_v_los_positive(self):
        reward_computer = RewardComputer(
            {
                "progress": 0.0,
                "heading_align": 0.0,
                "obstacle_proximity": 0.0,
                "action_rate": 0.0,
                "cross_track": 0.0,
                "step_penalty": 0.0,
                "forward_velocity": 0.5,
                "stall_penalty": 0.0,
                "terminal": 1.0,
            }
        )

        reward, components = reward_computer.compute(
            state={"dt": 0.25, "v_los": 2.0},
            prev_state={},
            action=[0.0, 0.0],
            prev_action=[0.0, 0.0],
        )

        self.assertAlmostEqual(components["forward_velocity"], 1.0)
        self.assertGreater(reward, 0.0)

    def test_stall_penalty_activates_only_after_warmup(self):
        reward_computer = RewardComputer(
            {
                "progress": 0.0,
                "heading_align": 0.0,
                "obstacle_proximity": 0.0,
                "action_rate": 0.0,
                "cross_track": 0.0,
                "step_penalty": 0.0,
                "forward_velocity": 0.0,
                "stall_penalty": 0.1,
                "stall_speed_threshold": 0.3,
                "stall_warmup_seconds": 10.0,
                "terminal": 1.0,
            }
        )

        _, pre_warmup_components = reward_computer.compute(
            state={"dt": 0.25, "speed": 0.2, "elapsed_time": 10.0},
            prev_state={},
            action=[0.0, 0.0],
            prev_action=[0.0, 0.0],
        )
        _, post_warmup_components = reward_computer.compute(
            state={"dt": 0.25, "speed": 0.2, "elapsed_time": 10.1},
            prev_state={},
            action=[0.0, 0.0],
            prev_action=[0.0, 0.0],
        )

        self.assertAlmostEqual(pre_warmup_components["stall_penalty"], 0.0)
        self.assertAlmostEqual(post_warmup_components["stall_penalty"], -0.1)

    def test_build_reward_state_includes_dt_and_motion_inputs_for_real_env_path(self):
        stub_env = SimpleNamespace(
            state={
                "distance_to_goal_x": 3.0,
                "distance_to_goal_y": 4.0,
                "heading_error": 0.2,
                "collision": False,
                "v_surge": 0.8,
                "v_los": 1.5,
                "speed": 0.4,
            },
            min_obstacle_distance=7.5,
            cross_track_error=-0.3,
            step_sleep=0.2,
            action_repeat=3,
            episode_start_time=100.0,
        )

        with mock.patch("airgym.envs.vessel_env.time.time", return_value=107.5):
            reward_state = PCGVesselEnv._build_reward_state(stub_env, goal_reached=True)

        self.assertAlmostEqual(reward_state["distance_to_goal"], 5.0)
        self.assertAlmostEqual(reward_state["dt"], 0.6)
        self.assertAlmostEqual(reward_state["v_surge"], 0.8)
        self.assertAlmostEqual(reward_state["v_los"], 1.5)
        self.assertAlmostEqual(reward_state["speed"], 0.4)
        self.assertAlmostEqual(reward_state["elapsed_time"], 7.5)
        self.assertTrue(reward_state["goal_reached"])
        self.assertFalse(reward_state["collision"])


if __name__ == "__main__":
    unittest.main()
