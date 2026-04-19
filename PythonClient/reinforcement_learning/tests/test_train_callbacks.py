from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import train


class _FakeTrainingEnv:
    def __init__(self):
        self.saved_paths = []
        self.reset_calls = 0

    def save(self, path):
        self.saved_paths.append(path)

    def reset(self):
        self.reset_calls += 1
        return "reset-observation"


class _FakeModel:
    def __init__(self, env, logger=None):
        self._env = env
        self._last_obs = "previous-observation"
        self.logger = logger if logger is not None else _FakeLogger()

    def get_env(self):
        return self._env


class _FakeLogger:
    def __init__(self):
        self.records = {}

    def record(self, key, value, *args, **kwargs):
        self.records[key] = value


class EpisodeEndCallbackTests(unittest.TestCase):
    def test_episode_end_callback_records_explicit_terminal_and_motion_metrics(self):
        callback = train.EpisodeEndCallback(use_wandb=False)
        model = _FakeModel(_FakeTrainingEnv(), logger=_FakeLogger())
        callback.model = model
        callback.num_timesteps = 321

        callback.locals = {
            "actions": [[0.2, 0.1]],
            "infos": [
                {
                    "reward_components": {"progress": 1.25},
                    "v_surge": 0.4,
                    "speed": 0.0,
                }
            ],
        }
        self.assertTrue(callback._on_step())
        self.assertEqual(model.logger.records, {})

        callback.locals = {
            "actions": [[0.4, -0.2]],
            "infos": [
                {
                    "end_reason": "timeout",
                    "distance_to_final_goal": 12.5,
                    "distance_to_current_wp": 3.5,
                    "waypoints_reached": 1,
                    "episode_num": 7,
                    "path_length_ratio": 1.2,
                    "curriculum_stage": 2,
                    "reward_components": {"safety": -0.5},
                    "v_surge": 0.6,
                    "speed": 1.0,
                }
            ],
        }

        self.assertTrue(callback._on_step())

        self.assertEqual(model.logger.records["episode/timeout"], 1)
        self.assertEqual(model.logger.records["episode/goal_reached"], 0)
        self.assertAlmostEqual(model.logger.records["episode/final_distance_to_goal"], 12.5)
        self.assertAlmostEqual(model.logger.records["episode/final_distance_to_current_wp"], 3.5)
        self.assertAlmostEqual(model.logger.records["episode/mean_v_surge"], 0.5)
        self.assertAlmostEqual(model.logger.records["episode/time_moving_frac"], 0.5)
        self.assertAlmostEqual(model.logger.records["episode/mean_thrust"], 0.3)
        self.assertAlmostEqual(model.logger.records["episode/mean_yaw_cmd"], -0.05)
        self.assertAlmostEqual(model.logger.records["reward_components/progress"], 1.25)
        self.assertAlmostEqual(model.logger.records["reward_components/safety"], -0.5)


class EvalSuiteCallbackTests(unittest.TestCase):
    def test_eval_suite_callback_does_not_reset_training_env_after_terminal_eval(self):
        config = SimpleNamespace(
            train=SimpleNamespace(
                eval_freq=100,
                subset_eval_seeds=2,
                subset_eval_episodes=3,
                deterministic_eval=True,
            )
        )
        callback = train.EvalSuiteCallback(
            config=config,
            run_dir=Path("E:/code/ASVSim/data/reinforcement_learning/runs/test-run"),
            vecnormalize_path=Path("E:/code/ASVSim/data/reinforcement_learning/runs/test-run/vecnorm.pkl"),
        )
        training_env = _FakeTrainingEnv()
        model = _FakeModel(training_env)

        callback.model = model
        callback.num_timesteps = 100
        callback.locals = {
            "infos": [{"end_reason": "goal_reached"}],
            "new_obs": "step-observation",
        }

        with mock.patch("eval_suite.run_eval_suite") as run_eval_suite:
            result = callback._on_step()

        self.assertTrue(result)
        run_eval_suite.assert_called_once()
        self.assertEqual(training_env.saved_paths, [str(callback.vecnormalize_path)])
        self.assertEqual(training_env.reset_calls, 0)
        self.assertEqual(model._last_obs, "previous-observation")
        self.assertEqual(callback.locals["new_obs"], "step-observation")
        self.assertEqual(callback.next_eval, 200)


if __name__ == "__main__":
    unittest.main()
