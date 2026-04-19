from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import pandas as pd

import eval_suite
from airgym.envs.vessel_env import PCGVesselEnv


class _FakeBaseEnv:
    def __init__(self):
        self.episode_params = []

    def set_next_episode_params(self, **kwargs):
        self.episode_params.append(kwargs)


class _FakeVecEnv:
    def __init__(self):
        self.envs = [_FakeBaseEnv()]
        self.seed_calls = []
        self.reset_calls = 0
        self.closed = False

    def seed(self, seed):
        self.seed_calls.append(seed)

    def reset(self):
        self.reset_calls += 1
        return np.zeros((1, 1), dtype=np.float32)

    def step(self, action):
        info = {
            "end_reason": "goal_reached",
            "distance_to_final_goal": 9.0,
            "distance_to_goal_x": 3.0,
            "distance_to_goal_y": 4.0,
            "path_length_ratio": 1.0,
        }
        return (
            np.zeros((1, 1), dtype=np.float32),
            np.array([1.0], dtype=np.float32),
            np.array([True]),
            [info],
        )

    def close(self):
        self.closed = True


class _FakeModel:
    def predict(self, obs, deterministic=True):
        return np.zeros((1, 2), dtype=np.float32), None


class EvalSuiteSemanticsTests(unittest.TestCase):
    def test_pcg_vessel_env_sim_crash_uses_last_known_distances(self):
        env = PCGVesselEnv.__new__(PCGVesselEnv)
        env.sim_launch_mode = "exe"
        env.single_obs_size = 54
        env.n_stack = 1
        env.episode_count = 11
        env.timestep = 3
        env._restart_sim = lambda: None
        env._step_inner = mock.Mock(side_effect=RuntimeError("sim blew up"))
        env.state = {
            "vessel_state": object(),
            "distance_to_final_goal": 17.5,
            "distance_to_current_wp": 4.25,
            "v_surge": 0.6,
            "v_los": 0.4,
            "speed": 0.8,
        }

        _, _, terminated, truncated, info = env.step(np.array([0.0, 0.0], dtype=np.float32))

        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["end_reason"], "sim_crash")
        self.assertAlmostEqual(info["distance_to_final_goal"], 17.5)
        self.assertAlmostEqual(info["distance_to_current_wp"], 4.25)

    def test_pcg_vessel_env_sim_crash_uses_nan_without_last_known_distances(self):
        env = PCGVesselEnv.__new__(PCGVesselEnv)
        env.sim_launch_mode = "exe"
        env.single_obs_size = 54
        env.n_stack = 1
        env.episode_count = 12
        env.timestep = 0
        env._restart_sim = lambda: None
        env._step_inner = mock.Mock(side_effect=RuntimeError("sim blew up"))
        env.state = {
            "vessel_state": None,
        }

        _, _, terminated, truncated, info = env.step(np.array([0.0, 0.0], dtype=np.float32))

        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["end_reason"], "sim_crash")
        self.assertTrue(np.isnan(info["distance_to_final_goal"]))
        self.assertTrue(np.isnan(info["distance_to_current_wp"]))

    def test_run_eval_suite_reseeds_vec_env_for_each_seed_group(self):
        config = SimpleNamespace(
            train=SimpleNamespace(
                seed=43,
                full_eval_seeds=2,
                full_eval_episodes=2,
                deterministic_eval=True,
            )
        )
        fake_env = _FakeVecEnv()
        fake_model = _FakeModel()
        stages = [
            {
                "name": "stage_0",
                "num_obstacles": 1,
                "num_dynamic_obstacles": 0,
                "num_waypoints": 1,
                "length": 10,
                "angle_range": [-45.0, 45.0],
            }
        ]

        with TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            with (
                mock.patch.object(eval_suite, "create_eval_env", return_value=fake_env),
                mock.patch.object(eval_suite, "build_stage_specs", return_value=stages),
                mock.patch.object(eval_suite, "dump_trajectory"),
            ):
                results = eval_suite.run_eval_suite(
                    config=config,
                    run_dir=run_dir,
                    model=fake_model,
                    output_csv=run_dir / "results.csv",
                    output_dir=run_dir,
                    num_seeds=2,
                    episodes_per_seed=2,
                    deterministic=True,
                )
                self.assertTrue((results["results"]["final_dist"] == 9.0).all())

        self.assertEqual(fake_env.seed_calls, [43, 44])
        self.assertEqual(fake_env.reset_calls, 4)
        self.assertTrue(fake_env.closed)

    def test_summarise_results_includes_sim_crash_rate(self):
        results_df = pd.DataFrame(
            [
                {
                    "stage": "base",
                    "end_reason": "goal_reached",
                    "reward": 1.0,
                    "final_dist": 0.0,
                    "path_length_ratio": 1.0,
                },
                {
                    "stage": "base",
                    "end_reason": "sim_crash",
                    "reward": -1.0,
                    "final_dist": 10.0,
                    "path_length_ratio": 2.0,
                },
                {
                    "stage": "advanced",
                    "end_reason": "collision",
                    "reward": -2.0,
                    "final_dist": 20.0,
                    "path_length_ratio": 3.0,
                },
                {
                    "stage": "advanced",
                    "end_reason": "timeout",
                    "reward": -3.0,
                    "final_dist": 30.0,
                    "path_length_ratio": 4.0,
                },
            ]
        )

        stage_summary, overall = eval_suite.summarise_results(results_df)

        self.assertIn("sim_crash_rate", stage_summary.columns)
        base_row = stage_summary.loc[stage_summary["stage"] == "base"].iloc[0]
        self.assertAlmostEqual(base_row["sim_crash_rate"], 0.5)
        self.assertAlmostEqual(overall["sim_crash_rate"], 0.25)


if __name__ == "__main__":
    unittest.main()
