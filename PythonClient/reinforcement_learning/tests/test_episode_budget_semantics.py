import unittest

from config import DEFAULT_CONFIG, load_config, resolve_env_max_timesteps, validate_config


class EpisodeBudgetSemanticsTests(unittest.TestCase):
    def test_default_budget_uses_first_waypoint_budget(self):
        config = load_config(DEFAULT_CONFIG)
        validate_config(config)

        self.assertIsNone(config.env.max_timesteps)
        self.assertEqual(int(config.env.first_waypoint_max_timesteps), 1200)
        self.assertEqual(int(config.env.additional_waypoint_max_timesteps), 800)
        self.assertEqual(resolve_env_max_timesteps(config.env), 1200)

    def test_default_budget_adds_increment_for_additional_waypoints(self):
        config = load_config(DEFAULT_CONFIG, ["env.num_waypoints=3"])
        validate_config(config)

        self.assertEqual(resolve_env_max_timesteps(config.env), 2800)

    def test_explicit_max_timesteps_override_wins(self):
        config = load_config(DEFAULT_CONFIG, ["env.num_waypoints=3", "env.max_timesteps=900"])
        validate_config(config)

        self.assertEqual(resolve_env_max_timesteps(config.env), 900)


if __name__ == "__main__":
    unittest.main()
