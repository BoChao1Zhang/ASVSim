import unittest

from airgym.envs.reward import RewardComputer
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


if __name__ == "__main__":
    unittest.main()
