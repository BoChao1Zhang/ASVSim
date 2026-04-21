import math
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np
from gymnasium import spaces

from airgym.envs.vessel_env import (
    PCGVesselEnv,
    project_velocity_onto_los,
    world_velocity_to_body_frame,
)


def _make_vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x_val=x, y_val=y, z_val=z)


class _FakeVessel:
    def __init__(self, vessel_state, lidar_data, collision_info):
        self._vessel_state = vessel_state
        self._lidar_data = lidar_data
        self._collision_info = collision_info

    def getVesselState(self):
        return self._vessel_state

    def getLidarData(self):
        return self._lidar_data

    def simGetCollisionInfo(self):
        return self._collision_info


class _ActionRecordingVessel:
    def __init__(self):
        self.calls = []

    def setVesselControls(self, vehicle_name, vessel_controls):
        self.calls.append((vehicle_name, vessel_controls))


class _InitStubVessel:
    def confirmConnection(self):
        return None

    def enableApiControl(self, is_enabled):
        return None

    def isApiControlEnabled(self):
        return True

    def armDisarm(self, is_armed):
        return True

    def getVesselState(self):
        return None

    def setVesselControls(self, vehicle_name, vessel_controls):
        return None

    def activateGeneration(self, landscape):
        return True


class ObservationSemanticsTests(unittest.TestCase):
    def test_default_action_space_uses_expanded_rudder_window(self):
        with mock.patch("airgym.envs.vessel_env.airsim.VesselClient", return_value=_InitStubVessel()):
            env = PCGVesselEnv(step_sleep=0.0)

        np.testing.assert_allclose(
            env.action_space.low,
            np.array([0.0, 0.44055554], dtype=np.float32),
            rtol=1e-6,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            env.action_space.high,
            np.array([0.7, 0.5594444], dtype=np.float32),
            rtol=1e-6,
            atol=1e-6,
        )

    def test_world_velocity_to_body_frame_rotates_into_surge_and_sway(self):
        surge, sway = world_velocity_to_body_frame(
            world_velocity_x=3.0,
            world_velocity_y=4.0,
            heading=math.pi / 2.0,
        )

        self.assertAlmostEqual(surge, 4.0)
        self.assertAlmostEqual(sway, -3.0)

    def test_project_velocity_onto_los_returns_scalar_projection(self):
        projection = project_velocity_onto_los(
            world_velocity_x=3.0,
            world_velocity_y=0.0,
            los_dx=4.0,
            los_dy=3.0,
        )

        self.assertAlmostEqual(projection, 2.4)

    def test_get_obs_emits_body_frame_velocity_entries_without_shape_change(self):
        env = object.__new__(PCGVesselEnv)
        env.waypoints = [(10.0, 0.0)]
        env.current_waypoint_idx = 0
        env.state = {
            "position": np.zeros(3, dtype=np.float32),
            "prev_position": np.zeros(3, dtype=np.float32),
            "collision": False,
        }
        env.timestep = 2
        env.prev_waypoint_x = 0.0
        env.prev_waypoint_y = 0.0
        env.prev_actions = np.array([0.0, 0.0], dtype=np.float32)
        env.rng = np.random.RandomState(0)
        env._reset_collision_ts = 0
        env.FILTER_LABELS = set()

        heading = math.pi / 2.0
        yaw = -heading
        vessel_state = SimpleNamespace(
            kinematics_estimated=SimpleNamespace(
                position=_make_vector(0.0, 0.0, 0.0),
                orientation=SimpleNamespace(
                    x_val=0.0,
                    y_val=0.0,
                    z_val=math.sin(yaw / 2.0),
                    w_val=math.cos(yaw / 2.0),
                ),
                linear_velocity=_make_vector(1.0, 0.0, 0.0),
                linear_acceleration=_make_vector(0.0, 0.0, 0.0),
                angular_acceleration=_make_vector(0.0, 0.0, 0.0),
            )
        )
        lidar_data = SimpleNamespace(
            point_cloud=np.zeros(3600 * 3, dtype=np.float32),
            groundtruth=[b"None"] * 3600,
        )
        collision_info = SimpleNamespace(has_collided=False, time_stamp=0)
        env.vessel = _FakeVessel(vessel_state, lidar_data, collision_info)

        obs = env._get_obs()

        self.assertEqual(obs.shape, (54,))
        self.assertAlmostEqual(obs[11], 0.0, places=6)
        self.assertAlmostEqual(obs[12], -1.0, places=6)

    def test_get_obs_requires_exact_3600_lidar_points_for_sector_pooling(self):
        env = object.__new__(PCGVesselEnv)
        env.waypoints = [(10.0, 0.0)]
        env.current_waypoint_idx = 0
        env.state = {
            "position": np.zeros(3, dtype=np.float32),
            "prev_position": np.zeros(3, dtype=np.float32),
            "collision": False,
        }
        env.timestep = 2
        env.prev_waypoint_x = 0.0
        env.prev_waypoint_y = 0.0
        env.prev_actions = np.array([0.0, 0.0], dtype=np.float32)
        env.rng = np.random.RandomState(0)
        env._reset_collision_ts = 0
        env.FILTER_LABELS = set()

        vessel_state = SimpleNamespace(
            kinematics_estimated=SimpleNamespace(
                position=_make_vector(0.0, 0.0, 0.0),
                orientation=SimpleNamespace(
                    x_val=0.0,
                    y_val=0.0,
                    z_val=0.0,
                    w_val=1.0,
                ),
                linear_velocity=_make_vector(1.0, 0.0, 0.0),
                linear_acceleration=_make_vector(0.0, 0.0, 0.0),
                angular_acceleration=_make_vector(0.0, 0.0, 0.0),
            )
        )
        lidar_data = SimpleNamespace(
            point_cloud=np.zeros(450 * 3, dtype=np.float32),
            groundtruth=[b"None"] * 450,
        )
        collision_info = SimpleNamespace(has_collided=False, time_stamp=0)
        env.vessel = _FakeVessel(vessel_state, lidar_data, collision_info)

        with self.assertRaisesRegex(ValueError, "3600"):
            env._get_obs()

    def test_do_action_matches_upstream_single_thruster_rudder_mapping(self):
        env = object.__new__(PCGVesselEnv)
        env.vessel = _ActionRecordingVessel()
        env.action_repeat = 1
        env.step_sleep = 0.0
        env.episode_count = 0
        env.timestep = 0
        env.prev_actions = np.array([0.0, 0.0], dtype=np.float32)
        env.action_space = spaces.Box(
            low=np.array([0.0, 0.44055554], dtype=np.float32),
            high=np.array([0.7, 0.5594444], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

        action = env._clip_action(np.array([1.0, 0.0], dtype=np.float32))

        np.testing.assert_allclose(
            action,
            np.array([0.7, 0.44055554], dtype=np.float32),
            rtol=1e-6,
            atol=1e-6,
        )

        env._do_action(np.array([0.7, 0.5594444], dtype=np.float32))

        self.assertEqual(len(env.vessel.calls), 1)
        _, vessel_controls = env.vessel.calls[0]
        self.assertAlmostEqual(vessel_controls.thruster_forces[0], 0.0)
        self.assertAlmostEqual(vessel_controls.thruster_forces[1], 0.7)
        self.assertAlmostEqual(vessel_controls.thruster_angles[0], 0.0)
        self.assertAlmostEqual(vessel_controls.thruster_angles[1], 0.5594444, places=6)
        np.testing.assert_allclose(
            env.prev_actions,
            np.array([0.7, 0.5594444], dtype=np.float32),
            rtol=1e-6,
            atol=1e-6,
        )


if __name__ == "__main__":
    unittest.main()
