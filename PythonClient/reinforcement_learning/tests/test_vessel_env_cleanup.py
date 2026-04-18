import unittest

from airgym.envs.vessel_env import PCGVesselEnv


class _FakeVessel:
    def __init__(self, failing_names=None):
        self.failing_names = set(failing_names or [])
        self.destroy_calls = []
        self.false_names = set()

    def simDestroyObject(self, name):
        self.destroy_calls.append(name)
        if name in self.failing_names:
            raise RuntimeError(f"failed to destroy {name}")
        if name in self.false_names:
            return False
        return True


class DestroyObstaclesTests(unittest.TestCase):
    def test_destroy_obstacles_keeps_false_return_names_for_retry(self):
        env = object.__new__(PCGVesselEnv)
        env.use_c_side_pcg_obstacles = False
        env.vessel = _FakeVessel()
        env.vessel.false_names = {"obs_b"}
        env.spawned_obstacle_names = ["obs_a", "obs_b", "obs_c"]

        env._destroy_obstacles()

        self.assertEqual(env.vessel.destroy_calls, ["obs_a", "obs_b", "obs_c"])
        self.assertEqual(env.spawned_obstacle_names, ["obs_b"])

    def test_destroy_obstacles_keeps_failed_names_for_retry(self):
        env = object.__new__(PCGVesselEnv)
        env.use_c_side_pcg_obstacles = False
        env.vessel = _FakeVessel(failing_names={"obs_b"})
        env.spawned_obstacle_names = ["obs_a", "obs_b", "obs_c"]

        env._destroy_obstacles()

        self.assertEqual(env.vessel.destroy_calls, ["obs_a", "obs_b", "obs_c"])
        self.assertEqual(env.spawned_obstacle_names, ["obs_b"])


if __name__ == "__main__":
    unittest.main()
