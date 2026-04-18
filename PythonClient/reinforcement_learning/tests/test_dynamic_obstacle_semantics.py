import unittest

from config import DEFAULT_CONFIG, load_config, validate_config


class DynamicObstacleSemanticsTests(unittest.TestCase):
    def test_validate_config_rejects_dynamic_obstacle_experiment_dimension(self):
        config = load_config(DEFAULT_CONFIG)
        config.env.num_dynamic_obstacles = 1

        with self.assertRaisesRegex(ValueError, "num_dynamic_obstacles"):
            validate_config(config)

        config = load_config(DEFAULT_CONFIG)
        self.assertGreater(len(config.curriculum.stages), 0)
        config.curriculum.stages[0].num_dynamic_obstacles = 1

        with self.assertRaisesRegex(ValueError, "num_dynamic_obstacles"):
            validate_config(config)


if __name__ == "__main__":
    unittest.main()
