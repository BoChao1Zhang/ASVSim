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
    def __init__(self, env):
        self._env = env
        self._last_obs = "previous-observation"

    def get_env(self):
        return self._env


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
