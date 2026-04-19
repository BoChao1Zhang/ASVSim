import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import cosysairsim as airsim
from cosysairsim.types import VesselControls, Pose, Quaternionr, Vector3r, Vector2r
import numpy as np
import math
import time
import subprocess
from collections import deque

import gymnasium as gym
from gymnasium import spaces

from airgym.envs.reward import RewardComputer
from diagnostics import diag_log


def world_velocity_to_body_frame(world_velocity_x, world_velocity_y, heading):
    surge = world_velocity_x * math.cos(heading) + world_velocity_y * math.sin(heading)
    sway = -world_velocity_x * math.sin(heading) + world_velocity_y * math.cos(heading)
    return float(surge), float(sway)


def project_velocity_onto_los(world_velocity_x, world_velocity_y, los_dx, los_dy):
    los_norm = math.hypot(los_dx, los_dy)
    if los_norm <= 1e-9:
        return 0.0
    return float((world_velocity_x * los_dx + world_velocity_y * los_dy) / los_norm)


class PCGVesselEnv(gym.Env):
    """Gymnasium environment for vessel RL training with PCG terrain randomization."""

    def __init__(
        self,
        ip_address="127.0.0.1",
        terrain_regen_interval=10,
        num_obstacles=4,
        num_dynamic_obstacles=0,
        goal_distance=2,
        max_timesteps=800,
        step_sleep=0.25,
        action_repeat=1,
        seed=42,
        sim_path="Blocks/Blocks.exe",
        sim_wait=10,
        sim_launch_mode="none",
        use_c_side_pcg_obstacles=False,
        terrain_length=10,
        angle_range=None,
        terrain_min_width_cm=5000.0,
        terrain_max_width_cm=10000.0,
        reward_config=None,
        n_stack=1,
        lidar_noise_sigma=0.0,
        heading_noise_sigma=0.0,
        waypoint_radius=10.0,
        yaw_angle_scale=0.6,
    ):
        super().__init__()

        self.ip_address = ip_address
        self.sim_path = sim_path
        self.sim_wait = sim_wait
        self.sim_launch_mode = sim_launch_mode
        self.sim_proc = None  # managed externally or by _restart_sim
        self.terrain_regen_interval = terrain_regen_interval
        self.num_obstacles = num_obstacles
        self.num_dynamic_obstacles = num_dynamic_obstacles
        self.goal_distance = goal_distance
        self.max_timesteps = max_timesteps
        self.step_sleep = step_sleep
        self.action_repeat = action_repeat
        self.base_seed = seed
        # When True the native AGenerationManager::SpawnAdvancedObstacles path
        # already populates a navigable, mixed-class obstacle set; the Python
        # helpers below short-circuit and defer cleanup to the Blueprint's own
        # Clear() step (which destroys everything in the Generated array).
        # Default is False to preserve backward-compatible behavior for existing
        # training/eval scripts (eval_vessel.py, crossq_vessel.py) that only
        # pass num_obstacles / num_dynamic_obstacles.
        self.use_c_side_pcg_obstacles = use_c_side_pcg_obstacles
        self.terrain_length = int(terrain_length)
        self.angle_range = [-45.0, 45.0] if angle_range is None else [float(angle_range[0]), float(angle_range[1])]
        self.terrain_min_width_cm = float(terrain_min_width_cm)
        self.terrain_max_width_cm = float(terrain_max_width_cm)
        self.reward_computer = RewardComputer(reward_config or {})
        self.n_stack = int(n_stack)
        self.lidar_noise_sigma = float(lidar_noise_sigma)
        self.heading_noise_sigma = float(heading_noise_sigma)
        self.waypoint_radius = float(waypoint_radius)
        self.yaw_angle_scale = float(yaw_angle_scale)
        self.timestep = 0
        self.episode_count = 0
        self.rng = np.random.RandomState(seed)
        self._next_episode_params = {}
        self._forced_terrain_seed = None
        self._force_terrain_regen = False
        self.min_obstacle_distance = 999.0
        self.cross_track_error = 0.0
        self.prev_reward_state = None
        self.episode_start_time = None
        self.current_obstacles = []
        self.episode_trajectory = []
        self.episode_path_length = 0.0
        self.initial_final_goal_distance = 1.0

        # Goal and terrain state
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.border_points = []
        self.initial_location = Vector2r(0.0, 0.0)
        self.spawned_obstacle_names = []

        # Waypoint navigation state
        # waypoints: list of (x, y) tuples — [intermediate, final_goal]
        self.waypoints = []
        self.current_waypoint_idx = 0
        # Position of last reached waypoint (or vessel start position)
        self.prev_waypoint_x = 0.0
        self.prev_waypoint_y = 0.0

        # Collision timestamp baseline (ignore collisions from before reset)
        self._reset_collision_ts = 0

        # Previous state tracking
        self.prev_actions = np.zeros(2, dtype=np.float32)
        self.prev_distance_to_goal_x = 0.0
        self.prev_distance_to_goal_y = 0.0
        self._waypoint_start_distance = 1.0
        self._next_distance_milestone = None

        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
            "vessel_state": None,
            "success": False,
            "distance_to_goal_x": 0.0,
            "distance_to_goal_y": 0.0,
            "distance_to_current_wp": 0.0,
            "distance_to_final_goal": 0.0,
            "heading": 0.0,
            "heading_error": 0.0,
            "cross_track_error": 0.0,
            "v_surge": 0.0,
            "v_los": 0.0,
            "speed": 0.0,
        }


        # Action space: [thrust, yaw_cmd]
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

        # Single-frame observation: 54-dim vector
        # 2 prev waypoint dist + 2 curr waypoint dist + 2 next waypoint dist
        # + 1 heading_error + 2 heading (sin, cos) + 2 velocity + 3 acceleration
        # + 2 prev actions (thrust, yaw_cmd)
        # + 36 lidar (3600 points min-pooled into 36 sectors of 10°)
        self.single_obs_size = 54

        # Frame stacking: concatenate n_stack frames
        self.frame_buffer = deque(maxlen=self.n_stack)
        self.observation_space = spaces.Box(
            low=float("-inf"),
            high=float("inf"),
            shape=(self.single_obs_size * self.n_stack,),
            dtype=np.float32,
        )

        # Labels to filter out of LiDAR (set to 0 = no obstacle)
        self.FILTER_LABELS = {
            b"Ground", b"milliampere",
            b"Landscape_0", b"Landscape_1", b"PCGVolume_1",
        }

        # Connect to simulator
        self.vessel = airsim.VesselClient(ip=ip_address)
        self._setup_vessel()

        # Activate PCG system once (landscape=False: landscape creation is editor-only)
        pcg_ok = self.vessel.activateGeneration(False)
        print(f"PCG activateGeneration returned: {pcg_ok}")
        if not pcg_ok:
            raise RuntimeError(
                "activateGeneration(False) failed. Ensure the target world is fully loaded and the PCG plugins are enabled."
            )
        # Terrain is generated on the first reset() call (episode 1)

    def set_next_episode_params(self, **kwargs):
        allowed = {
            "num_obstacles",
            "num_dynamic_obstacles",
            "goal_distance",
            "terrain_length",
            "angle_range",
            "terrain_seed",
            "force_terrain_regen",
        }
        for key, value in kwargs.items():
            if key not in allowed:
                raise ValueError(f"Unsupported episode override: {key}")
            if key == "terrain_seed":
                self._forced_terrain_seed = int(value)
            elif key == "force_terrain_regen":
                self._force_terrain_regen = bool(value)
            elif key == "angle_range":
                if len(value) != 2:
                    raise ValueError("angle_range override must contain exactly two values")
                self._next_episode_params[key] = [float(value[0]), float(value[1])]
            else:
                self._next_episode_params[key] = value
        diag_log(
            "env_set_next_episode_params",
            pending_params=self._next_episode_params,
            forced_terrain_seed=self._forced_terrain_seed,
            force_terrain_regen=self._force_terrain_regen,
        )

    def _apply_next_episode_params(self):
        for key, value in self._next_episode_params.items():
            if key == "goal_distance":
                self.goal_distance = int(value)
            elif key == "num_obstacles":
                self.num_obstacles = int(value)
            elif key == "num_dynamic_obstacles":
                self.num_dynamic_obstacles = int(value)
            elif key == "terrain_length":
                self.terrain_length = int(value)
            elif key == "angle_range":
                self.angle_range = [float(value[0]), float(value[1])]
            else:
                setattr(self, key, value)
        self._next_episode_params = {}

    def _wait_collision_clear(self, timeout=2.0):
        """Poll until collision info is cleared after reset (max timeout seconds)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.vessel.simGetCollisionInfo().has_collided:
                return
            time.sleep(0.05)

    def _setup_vessel(self):
        self.vessel.confirmConnection()
        print("Connected to AirSim Vessel")
        self.vessel.enableApiControl(True)
        if not self.vessel.isApiControlEnabled():
            raise RuntimeError("API control was not enabled for the vessel.")
        arm_ok = self.vessel.armDisarm(True)
        if arm_ok is False:
            raise RuntimeError("armDisarm(True) failed for the vessel.")
        self.vessel.getVesselState()
        self.vessel.setVesselControls("", VesselControls([0.0, 0.0], [0.5, 0.5]))

    def _restart_sim(self, max_retries=3):
        """Kill the simulator, restart it, reconnect, and re-activate PCG."""
        if self.sim_launch_mode != "exe":
            raise RuntimeError(
                "Automatic simulator restart is only available when sim_launch_mode='exe'. "
                "When attached to Unreal Editor, restart the runtime manually and rerun the script."
            )
        for attempt in range(max_retries):
            try:
                print(f"SIM RESTART: attempt {attempt + 1}/{max_retries}...")

                # Kill any existing sim process
                if self.sim_proc is not None:
                    self.sim_proc.kill()
                    self.sim_proc.wait()
                else:
                    os.system("taskkill /F /IM Blocks.exe >nul 2>&1")
                time.sleep(3.0)

                # Start fresh sim
                self.sim_proc = subprocess.Popen(
                    [self.sim_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                wait_time = self.sim_wait + attempt * 10  # wait longer on retries
                print(f"SIM RESTART: new PID={self.sim_proc.pid}, waiting {wait_time}s...")
                time.sleep(wait_time)

                # Reconnect client
                self.vessel = airsim.VesselClient(ip=self.ip_address)
                self._setup_vessel()

                # Re-activate PCG
                pcg_ok = self.vessel.activateGeneration(False)
                print(f"SIM RESTART: PCG activateGeneration returned: {pcg_ok}")

                # Force terrain regeneration on next reset
                self._needs_terrain_regen = True
                print("SIM RESTART: complete")
                return  # success

            except Exception as e:
                print(f"SIM RESTART: attempt {attempt + 1} failed: {e}")
                if attempt == max_retries - 1:
                    print("SIM RESTART: all retries exhausted, giving up")
                    raise

    def _generate_terrain(self):
        """Generate new PCG terrain and fetch waypoints for each section up to goal_distance."""
        if self._forced_terrain_seed is not None:
            seed = int(self._forced_terrain_seed)
            self._forced_terrain_seed = None
        elif self.terrain_regen_interval == 0:
            seed = -538846106
        else:
            seed = -464588337 + self.base_seed + self.episode_count
        print(f"Generating terrain with seed={seed}")
        diag_log(
            "env_generate_terrain_begin",
            episode_count=self.episode_count,
            terrain_seed=seed,
            goal_distance=int(self.goal_distance),
            terrain_length=int(self.terrain_length),
            angle_range=list(self.angle_range),
            use_c_side_pcg_obstacles=bool(self.use_c_side_pcg_obstacles),
        )

        time.sleep(2.0)  # let sim settle before PCG generation
        terrain_ok = self.vessel.generatePortTerrain(
            "port",
            seed,
            int(self.terrain_length),
            float(self.angle_range[0]),
            float(self.angle_range[1]),
            float(self.terrain_min_width_cm),
            float(self.terrain_max_width_cm),
            bool(self.use_c_side_pcg_obstacles),
        )
        print(f"generatePortTerrain returned: {terrain_ok}")

        # Poll until final waypoint (section goal_distance) is valid
        max_goal_distance = 2000.0
        final_result = []
        goal_valid = False
        for attempt in range(10):
            time.sleep(2.0)
            final_result = self.vessel.getGoal(self.initial_location, self.goal_distance)
            if len(final_result) >= 1:
                gx, gy = final_result[0]["x_val"], final_result[0]["y_val"]
                dist = math.sqrt(gx**2 + gy**2)
                if gx == 0.0 and gy == 0.0:
                    print(f"  getGoal({self.goal_distance}) returned (0,0), retrying... ({attempt + 1}/10)")
                elif dist > max_goal_distance:
                    print(f"  getGoal({self.goal_distance}) returned unreasonable coords ({gx:.1f}, {gy:.1f}), retrying... ({attempt + 1}/10)")
                else:
                    goal_valid = True
                    break

        if not goal_valid:
            raise RuntimeError(
                "PCG terrain generation failed because getGoal returned invalid coordinates after all retries. "
                "Check that the world is ready and the PCG plugins are enabled."
            )

        if len(final_result) >= 3:
            self.border_points = final_result[1:]

        # Fetch waypoints for each section (1 through goal_distance)
        self.waypoints = []
        for section in range(1, self.goal_distance + 1):
            result = self.vessel.getGoal(self.initial_location, section)
            if len(result) >= 1 and (result[0]["x_val"] != 0.0 or result[0]["y_val"] != 0.0):
                self.waypoints.append((result[0]["x_val"], result[0]["y_val"]))
            else:
                print(f"WARNING: getGoal section {section} returned (0,0), skipping")

        if not self.waypoints:
            raise RuntimeError("No valid waypoints returned from PCG terrain generation")

        self.goal_x = self.waypoints[-1][0]
        self.goal_y = self.waypoints[-1][1]
        wp_str = "  ".join(f"WP{i+1}=({x:.1f}, {y:.1f})" for i, (x, y) in enumerate(self.waypoints))
        print(f"{wp_str}")
        diag_log(
            "env_generate_terrain_complete",
            episode_count=self.episode_count,
            waypoint_count=len(self.waypoints),
            final_goal_x=float(self.goal_x),
            final_goal_y=float(self.goal_y),
        )

    def _spawn_obstacles(self):
        """Spawn random obstacles between vessel and goal."""
        if self.use_c_side_pcg_obstacles:
            # Native SpawnAdvancedObstacles already populated the Blueprint's
            # Generated array during generatePortTerrain. Do nothing here;
            # cleanup also stays off (handled by BP Clear on regen).
            return

        if self.goal_x == 0.0 and self.goal_y == 0.0:
            print("WARNING: goal is (0,0), skipping obstacle spawn to avoid physics explosion")
            return

        # Get current vessel position
        vessel_state = self.vessel.getVesselState()
        start_x = vessel_state.kinematics_estimated.position.x_val
        start_y = vessel_state.kinematics_estimated.position.y_val

        # Convert NED meters to UE centimeters
        goal_x_cm = self.goal_x * 100
        goal_y_cm = self.goal_y * 100
        start_x_cm = start_x * 100
        start_y_cm = start_y * 100

        # Snapshot existing obstacle-class actors so we can attribute only the
        # newly-spawned ones to this episode (preserves level-placed ships).
        baseline = self._snapshot_obstacle_names()

        for _ in range(self.num_obstacles):
            # Random fraction along the path (avoid placing too close to start/goal)
            frac = self.rng.uniform(0.15, 0.85)
            # Add lateral offset for variety
            lateral_offset = self.rng.uniform(-2000, 2000)

            obs_x = start_x_cm + frac * (goal_x_cm - start_x_cm)
            obs_y = start_y_cm + frac * (goal_y_cm - start_y_cm) + lateral_offset

            pose = Pose(Vector3r(obs_x, obs_y, 250), Quaternionr())
            self.vessel.simAddObstacle(pose, 0, "buoy")
            self.current_obstacles.append(
                {
                    "x": float(obs_x / 100.0),
                    "y": float(obs_y / 100.0),
                    "dynamic": False,
                    "speed": 0.0,
                }
            )

        # Record only the names that appeared since the baseline.
        after = self._snapshot_obstacle_names()
        for name in after - baseline:
            if name not in self.spawned_obstacle_names:
                self.spawned_obstacle_names.append(name)

    # Blueprint classes produced by the legacy RPC spawn path. Used ONLY for
    # a snapshot-diff to attribute newly created actors to this episode; we
    # never blindly destroy every match because the training level itself may
    # have pre-placed vessels of the same class.
    _OBSTACLE_NAME_PATTERNS = [
        ".*BP_BuoySpawn.*",
        ".*Boat_Blueprint.*",
        ".*Barge_Blueprint.*",
        ".*BP_CargoPawn.*",
        ".*BP_NPCSpawn.*",
    ]

    def _snapshot_obstacle_names(self):
        """Return the set of current scene actor names matching obstacle-class patterns."""
        found = set()
        for pattern in self._OBSTACLE_NAME_PATTERNS:
            try:
                found.update(self.vessel.simListSceneObjects(pattern))
            except Exception as exc:
                print(f"WARNING: simListSceneObjects({pattern}) failed: {exc}")
        return found

    def _destroy_obstacles(self):
        """Remove only the obstacles this environment explicitly spawned via RPC.

        When use_c_side_pcg_obstacles=True, the native SpawnAdvancedObstacles path
        owns the Generated[] array and BP Clear() destroys them during regen;
        Python must not interfere or the scene will be wiped after each episode.
        """
        if self.use_c_side_pcg_obstacles:
            return

        remaining_names = []
        for name in list(self.spawned_obstacle_names):
            try:
                destroyed = self.vessel.simDestroyObject(name)
                if not destroyed:
                    print(f"WARNING: simDestroyObject({name}) returned False; keeping obstacle tracked for retry.")
                    remaining_names.append(name)
            except Exception as exc:
                print(f"WARNING: simDestroyObject({name}) failed: {exc}")
                remaining_names.append(name)
        self.spawned_obstacle_names = remaining_names

    def _compute_cross_track_error(self, pos_x, pos_y):
        curr_wp = self.waypoints[self.current_waypoint_idx]
        start = np.array([self.prev_waypoint_x, self.prev_waypoint_y], dtype=np.float32)
        end = np.array([curr_wp[0], curr_wp[1]], dtype=np.float32)
        segment = end - start
        segment_norm = np.linalg.norm(segment)
        if segment_norm < 1e-6:
            return 0.0
        rel = np.array([pos_x, pos_y], dtype=np.float32) - start
        cross = segment[0] * rel[1] - segment[1] * rel[0]
        return float(cross / segment_norm)

    def _build_reward_state(self, goal_reached=False):
        dx = float(self.state["distance_to_goal_x"])
        dy = float(self.state["distance_to_goal_y"])
        episode_start_time = getattr(self, "episode_start_time", None)
        elapsed_time = 0.0 if episode_start_time is None else max(0.0, float(time.time() - episode_start_time))
        return {
            "distance_to_goal": float(math.sqrt(dx**2 + dy**2)),
            "heading_error": float(self.state.get("heading_error", 0.0)),
            "min_obstacle_distance": float(self.min_obstacle_distance),
            "cross_track_error": float(self.cross_track_error),
            "dt": float(self.step_sleep) * float(self.action_repeat),
            "v_surge": float(self.state.get("v_surge", 0.0)),
            "v_los": float(self.state.get("v_los", 0.0)),
            "speed": float(self.state.get("speed", 0.0)),
            "elapsed_time": elapsed_time,
            "collision": bool(self.state["collision"]),
            "goal_reached": bool(goal_reached),
        }

    def _compute_motion_diagnostics(self, linear_velocity_x, linear_velocity_y, heading, goal_angle):
        v_surge, _ = world_velocity_to_body_frame(linear_velocity_x, linear_velocity_y, heading)
        v_los = project_velocity_onto_los(
            linear_velocity_x,
            linear_velocity_y,
            math.cos(goal_angle),
            math.sin(goal_angle),
        )
        speed = float(math.sqrt(linear_velocity_x**2 + linear_velocity_y**2))
        return v_surge, v_los, speed

    def _get_last_known_state_scalar(self, key):
        value = getattr(self, "state", {}).get(key, math.nan)
        try:
            value = float(value)
        except (TypeError, ValueError):
            return float("nan")
        return value if math.isfinite(value) else float("nan")

    def _build_sim_crash_info(self):
        return {
            "reward": 0.0,
            "reward_components": {},
            "thrust": 0.0,
            "yaw_cmd": 0.0,
            "distance_to_final_goal": self._get_last_known_state_scalar("distance_to_final_goal"),
            "distance_to_current_wp": self._get_last_known_state_scalar("distance_to_current_wp"),
            "v_surge": self._get_last_known_state_scalar("v_surge"),
            "v_los": self._get_last_known_state_scalar("v_los"),
            "speed": self._get_last_known_state_scalar("speed"),
            "success": 0,
            "collision": 0,
            "episode_num": self.episode_count,
            "end_reason": "sim_crash",
            "waypoints_reached": 0,
            "path_length_ratio": 0.0,
        }

    def _retarget_current_waypoint(self):
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]
        curr_wp = self.waypoints[self.current_waypoint_idx]
        dx_curr = curr_wp[0] - pos_x
        dy_curr = curr_wp[1] - pos_y
        self.state["distance_to_goal_x"] = dx_curr
        self.state["distance_to_goal_y"] = dy_curr
        self.prev_distance_to_goal_x = dx_curr
        self.prev_distance_to_goal_y = dy_curr
        goal_angle = np.arctan2(dy_curr, dx_curr)
        heading = self.state.get("heading", 0.0)
        self.state["heading_error"] = (heading - goal_angle + np.pi) % (2 * np.pi) - np.pi
        self.cross_track_error = self._compute_cross_track_error(pos_x, pos_y)
        self.state["cross_track_error"] = self.cross_track_error

    def _spawn_dynamic_obstacles(self):
        """Spawn moving obstacles using the simulator's built-in movement."""
        # Temporarily disabled because moving obstacles currently destabilize RL
        # training/evaluation behavior. Keep the config surface intact so we can
        # re-enable this path later without rewriting the pipeline.
        if self.num_dynamic_obstacles > 0:
            print(
                f"Dynamic obstacle spawning is temporarily disabled; "
                f"requested={self.num_dynamic_obstacles}, forcing 0."
            )
            diag_log("env_dynamic_obstacles_disabled", requested=int(self.num_dynamic_obstacles))
        return

        if self.use_c_side_pcg_obstacles:
            # Dynamic obstacles (speed>0 entries in the native catalog) are already
            # handled by SpawnAdvancedObstacles; skip the RPC duplicate.
            return

        if self.num_dynamic_obstacles <= 0:
            return

        vessel_state = self.vessel.getVesselState()
        start_x = vessel_state.kinematics_estimated.position.x_val
        start_y = vessel_state.kinematics_estimated.position.y_val

        start_x_cm = start_x * 100
        start_y_cm = start_y * 100
        goal_x_cm = self.goal_x * 100
        goal_y_cm = self.goal_y * 100

        baseline = self._snapshot_obstacle_names()

        for _ in range(self.num_dynamic_obstacles):
            frac = self.rng.uniform(0.2, 0.8)
            lateral_offset = self.rng.uniform(-2000, 2000)

            obs_x = start_x_cm + frac * (goal_x_cm - start_x_cm)
            obs_y = start_y_cm + frac * (goal_y_cm - start_y_cm) + lateral_offset

            speed = float(self.rng.uniform(500, 1500))
            pose = Pose(Vector3r(obs_x, obs_y, 250), Quaternionr())
            self.vessel.simAddObstacle(pose, speed, "boat")
            self.current_obstacles.append(
                {
                    "x": float(obs_x / 100.0),
                    "y": float(obs_y / 100.0),
                    "dynamic": True,
                    "speed": speed,
                }
            )

        after = self._snapshot_obstacle_names()
        for name in after - baseline:
            if name not in self.spawned_obstacle_names:
                self.spawned_obstacle_names.append(name)

    def _get_obs(self):
        """Get vessel state and LiDAR observations."""
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            raise RuntimeError(
                "No active waypoint is available. Terrain generation likely did not produce valid goals."
            )

        vessel_state = self.vessel.getVesselState()
        self.state["vessel_state"] = vessel_state

        # Update position
        self.state["prev_position"] = self.state["position"]
        self.state["position"] = np.array([
            vessel_state.kinematics_estimated.position.x_val,
            vessel_state.kinematics_estimated.position.y_val,
            vessel_state.kinematics_estimated.position.z_val,
        ])
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]

        if self.timestep <= 1:
            print(f"  DEBUG pos=({pos_x:.1f}, {pos_y:.1f}), wp={self.waypoints[self.current_waypoint_idx]}")

        # LiDAR data
        lidar_data = self.vessel.getLidarData()
        pointcloud = np.array(lidar_data.point_cloud, dtype=np.float32)
        pointcloud = np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))
        groundtruth = np.array(lidar_data.groundtruth, dtype=np.dtype("S"))

        # Compute 2D distances
        lidar2D = pointcloud[:, 0:2]
        lidar_distances = np.linalg.norm(lidar2D, axis=1)

        # Filter out unwanted labels (set to 0 = no obstacle)
        filter_mask = np.array([l in self.FILTER_LABELS for l in groundtruth])
        lidar_distances[filter_mask] = 0.0

        # Min obstacle distance for reward (exclude filtered points)
        non_zero = lidar_distances[lidar_distances > 0]
        self.min_obstacle_distance = float(np.min(non_zero)) if len(non_zero) > 0 else 999.0

        # Min-pool 3600 points into 36 sectors (each 10°, 100 points)
        lidar_sectors = np.array_split(lidar_distances, 36)
        lidar_pooled = np.zeros(36, dtype=np.float32)
        for idx, sector in enumerate(lidar_sectors):
            valid = sector[sector > 0]
            lidar_pooled[idx] = float(np.min(valid)) if valid.size > 0 else 0.0
        if self.lidar_noise_sigma > 0.0:
            lidar_pooled = np.clip(
                lidar_pooled + self.rng.normal(0.0, self.lidar_noise_sigma, size=lidar_pooled.shape),
                a_min=0.0,
                a_max=None,
            ).astype(np.float32)

        # Heading from quaternion
        z = vessel_state.kinematics_estimated.orientation.z_val
        x = vessel_state.kinematics_estimated.orientation.x_val
        y = vessel_state.kinematics_estimated.orientation.y_val
        w = vessel_state.kinematics_estimated.orientation.w_val
        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        heading = (-np.arctan2(siny, cosy) + np.pi) % (2 * np.pi) - np.pi
        self.state["heading"] = heading

        # Current waypoint distances (active navigation target)
        curr_wp = self.waypoints[self.current_waypoint_idx]
        dx_curr = curr_wp[0] - pos_x
        dy_curr = curr_wp[1] - pos_y
        self.state["distance_to_goal_x"] = dx_curr
        self.state["distance_to_goal_y"] = dy_curr

        # Previous waypoint distances (where we came from)
        dx_prev = self.prev_waypoint_x - pos_x
        dy_prev = self.prev_waypoint_y - pos_y

        # Next waypoint distances (lookahead; zeros if no next waypoint)
        if self.current_waypoint_idx + 1 < len(self.waypoints):
            next_wp = self.waypoints[self.current_waypoint_idx + 1]
            dx_next = next_wp[0] - pos_x
            dy_next = next_wp[1] - pos_y
        else:
            dx_next, dy_next = 0.0, 0.0

        # Scalar distances to current and next waypoints
        distance_to_curr = np.sqrt(dx_curr**2 + dy_curr**2)
        distance_to_next = np.sqrt(dx_next**2 + dy_next**2) if dx_next != 0.0 or dy_next != 0.0 else 0.0
        final_wp = self.waypoints[-1]
        distance_to_final = np.sqrt((final_wp[0] - pos_x)**2 + (final_wp[1] - pos_y)**2)
        self.state["distance_to_current_wp"] = float(distance_to_curr)
        self.state["distance_to_final_goal"] = float(distance_to_final)

        # Heading error to current waypoint, normalized to [-pi, pi]
        goal_angle = np.arctan2(dy_curr, dx_curr)
        heading_error = (heading - goal_angle + np.pi) % (2 * np.pi) - np.pi
        self.state["heading_error"] = heading_error

        observed_heading = heading
        observed_heading_error = heading_error
        if self.heading_noise_sigma > 0.0:
            observed_heading = (heading + self.rng.normal(0.0, self.heading_noise_sigma) + np.pi) % (2 * np.pi) - np.pi
            observed_heading_error = (observed_heading - goal_angle + np.pi) % (2 * np.pi) - np.pi

        # Velocities and accelerations
        linear_velocity_x = vessel_state.kinematics_estimated.linear_velocity.x_val
        linear_velocity_y = vessel_state.kinematics_estimated.linear_velocity.y_val
        linear_acceleration_x = vessel_state.kinematics_estimated.linear_acceleration.x_val
        linear_acceleration_y = vessel_state.kinematics_estimated.linear_acceleration.y_val
        angular_acceleration_z = vessel_state.kinematics_estimated.angular_acceleration.z_val
        body_frame_surge, body_frame_sway = world_velocity_to_body_frame(
            linear_velocity_x,
            linear_velocity_y,
            heading,
        )
        v_surge, v_los, speed = self._compute_motion_diagnostics(
            linear_velocity_x,
            linear_velocity_y,
            heading,
            goal_angle,
        )
        self.state["v_surge"] = v_surge
        self.state["v_los"] = v_los
        self.state["speed"] = speed
        self.cross_track_error = self._compute_cross_track_error(pos_x, pos_y)
        self.state["cross_track_error"] = self.cross_track_error

        obs = np.concatenate([
            np.array([
                dx_prev, dy_prev,
                dx_curr, dy_curr,
                distance_to_curr,
                dx_next, dy_next,
                distance_to_next,
                observed_heading_error,
                np.sin(observed_heading), np.cos(observed_heading),
                body_frame_surge, body_frame_sway,
                linear_acceleration_x, linear_acceleration_y,
                angular_acceleration_z,
            ]),
            self.prev_actions.flatten(),
            lidar_pooled.flatten(),
        ])

        self.prev_distance_to_goal_x = dx_curr
        self.prev_distance_to_goal_y = dy_curr

        # Only count collisions that happened after the current reset.
        col_info = self.vessel.simGetCollisionInfo()
        self.state["collision"] = (
            col_info.has_collided and col_info.time_stamp > self._reset_collision_ts
        )

        return obs.astype(np.float32)

    def _get_stacked_obs(self):
        """Get current observation and return frame-stacked version."""
        obs = self._get_obs()
        self.frame_buffer.append(obs)
        return np.concatenate(list(self.frame_buffer))

    def _clip_action(self, action):
        action = np.asarray(action, dtype=np.float32)
        thrust = float(np.clip(action[0], self.action_space.low[0], self.action_space.high[0]))
        yaw_cmd = float(np.clip(action[1], self.action_space.low[1], self.action_space.high[1]))
        return np.array([thrust, yaw_cmd], dtype=np.float32)

    def _do_action(self, action):
        thrust = float(action[0])
        yaw_cmd = float(action[1])
        angle_delta = 0.5 * yaw_cmd * self.yaw_angle_scale
        angle_stern = float(np.clip(0.5 - angle_delta, 0.0, 1.0))
        angle_bow = float(np.clip(0.5 + angle_delta, 0.0, 1.0))
        vessel_controls = VesselControls([thrust, thrust], [angle_stern, angle_bow])
        diag_log(
            "env_action_applied",
            episode_count=self.episode_count,
            timestep=self.timestep,
            thrust=thrust,
            yaw_cmd=yaw_cmd,
            angle_stern=angle_stern,
            angle_bow=angle_bow,
            action_repeat=int(self.action_repeat),
        )
        for _ in range(self.action_repeat):
            self.vessel.setVesselControls("", vessel_controls)
            time.sleep(self.step_sleep)
        self.prev_actions = np.asarray(action, dtype=np.float32).copy()

    def _compute_reward(self, action, prev_action):
        current_state = self._build_reward_state(goal_reached=False)
        distance = current_state["distance_to_goal"]
        terminated = False
        truncated = False
        if self.state["collision"]:
            terminated = True
            self.state["success"] = False
            current_state["collision"] = True
        elif distance < self.waypoint_radius:
            is_final = self.current_waypoint_idx == len(self.waypoints) - 1
            if is_final:
                terminated = True
                self.state["success"] = True
                current_state["goal_reached"] = True
            else:
                # Intermediate waypoint reached; advance to the next waypoint.
                reached = self.waypoints[self.current_waypoint_idx]
                self.prev_waypoint_x = reached[0]
                self.prev_waypoint_y = reached[1]
                self.current_waypoint_idx += 1
                self._retarget_current_waypoint()
        elif self.timestep >= self.max_timesteps:
            truncated = True
            self.state["success"] = False

        reward, components = self.reward_computer.compute(current_state, self.prev_reward_state or current_state, action, prev_action)
        if terminated or truncated:
            self.prev_reward_state = current_state
        else:
            self.prev_reward_state = self._build_reward_state(goal_reached=False)
        return reward, components, terminated, truncated

    def step(self, action):
        try:
            return self._step_inner(action)
        except Exception as e:
            print(f"SIM ERROR in step(): {e}")
            diag_log("env_step_exception", episode_count=self.episode_count, timestep=self.timestep, error=str(e))
            if self.sim_launch_mode == "exe":
                self._restart_sim()
                obs = np.zeros(self.single_obs_size * self.n_stack, dtype=np.float32)
                info = self._build_sim_crash_info()
                return obs, 0.0, False, True, info
            raise RuntimeError(
                "step() failed while attached to an external simulator/editor. "
                "Restart the UE Editor or fix the runtime state, then rerun training."
            ) from e

    def _step_inner(self, action):
        self.timestep += 1
        action = self._clip_action(action)
        prev_action = self.prev_actions.copy()
        self._do_action(action)
        obs = self._get_stacked_obs()
        step_path = np.linalg.norm(self.state["position"][:2] - self.state["prev_position"][:2])
        self.episode_path_length += float(step_path)
        self.episode_trajectory.append(self.state["position"][:2].copy())
        reward, components, terminated, truncated = self._compute_reward(action, prev_action)

        end_reason = None
        if terminated:
            end_reason = "collision" if self.state["collision"] else "goal_reached"
        elif truncated:
            end_reason = "timeout"

        # Distance to final goal (always waypoints[-1]) for logging
        final_wp = self.waypoints[-1]
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]
        final_dist_x = final_wp[0] - pos_x
        final_dist_y = final_wp[1] - pos_y
        path_length_ratio = self.episode_path_length / max(self.initial_final_goal_distance, 1e-6)

        # How many waypoints were fully reached this episode
        waypoints_reached = (
            len(self.waypoints) if end_reason == "goal_reached"
            else self.current_waypoint_idx
        )

        if end_reason is not None:
            try:
                # Preserve the terminal vessel state until the next env.reset().
                # Episode teardown should not hard-reset the simulator inside step().
                self.vessel.setVesselControls("", VesselControls([0.0, 0.0], [0.5, 0.5]))
            except Exception:
                pass  # will be handled by next reset()
            final_dist = np.sqrt(final_dist_x**2 + final_dist_y**2)
            print(f"[Episode {self.episode_count}] End: {end_reason} | wps={waypoints_reached}/{len(self.waypoints)} | reward={reward:.1f} | steps={self.timestep} | final_dist={final_dist:.1f} | initial_dist={self.initial_goal_distance:.1f}")
            diag_log(
                "env_episode_end",
                episode_count=self.episode_count,
                end_reason=end_reason,
                timestep=self.timestep,
                reward=float(reward),
                final_dist=float(final_dist),
                waypoints_reached=int(waypoints_reached),
                waypoint_count=len(self.waypoints),
                path_length_ratio=float(path_length_ratio),
                actual_path_length=float(self.episode_path_length),
                straight_line_distance=float(self.initial_final_goal_distance),
            )

        info = {
            "reward": reward,
            "reward_components": components,
            "thrust": action[0],
            "yaw_cmd": action[1],
            "distance_to_final_goal": float(self.state["distance_to_final_goal"]),
            "distance_to_current_wp": float(self.state["distance_to_current_wp"]),
            "v_surge": float(self.state["v_surge"]),
            "v_los": float(self.state["v_los"]),
            "speed": float(self.state["speed"]),
            "success": int(self.state["success"]),
            "collision": int(self.state["collision"]),
            "episode_num": self.episode_count,
            "end_reason": end_reason,
            "waypoints_reached": waypoints_reached,
            "path_length_ratio": float(path_length_ratio),
            "actual_path_length": float(self.episode_path_length),
            "straight_line_distance": float(self.initial_final_goal_distance),
        }
        if end_reason is not None:
            info["episode_trajectory"] = [point.tolist() for point in self.episode_trajectory]
            info["waypoints"] = [list(point) for point in self.waypoints]
            info["obstacles"] = list(self.current_obstacles)

        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        try:
            return self._reset_inner(seed=seed, options=options)
        except Exception as e:
            print(f"SIM ERROR in reset(): {e}")
            diag_log("env_reset_exception", episode_count=self.episode_count, error=str(e))
            if self.sim_launch_mode == "exe":
                self._restart_sim()
                return self._reset_inner(seed=seed, options=options)
            raise RuntimeError(
                "reset() failed while attached to an external simulator/editor. "
                "Ensure the target map is loaded, API control works, and PCG generation is available."
            ) from e

    def _reset_inner(self, seed=None, options=None):
        super().reset(seed=seed)
        requested_seed = seed
        if seed is not None:
            self.rng = np.random.RandomState(seed)
        self.timestep = 0
        self.episode_count += 1
        self._apply_next_episode_params()

        # Regenerate terrain: every N episodes, only on first episode if interval <= 0,
        # or forced after sim restart
        regen = (
            self._force_terrain_regen
            or getattr(self, '_needs_terrain_regen', False)
            or not self.waypoints
            or (
                self.episode_count == 1
                if self.terrain_regen_interval <= 0
                else (self.episode_count - 1) % self.terrain_regen_interval == 0
            )
        )
        self._needs_terrain_regen = False
        self._force_terrain_regen = False
        diag_log(
            "env_reset_begin",
            episode_count=self.episode_count,
            requested_seed=requested_seed,
            regen=bool(regen),
            terrain_regen_interval=int(self.terrain_regen_interval),
            current_num_obstacles=int(self.num_obstacles),
            current_num_dynamic_obstacles=int(self.num_dynamic_obstacles),
            current_goal_distance=int(self.goal_distance),
            current_terrain_length=int(self.terrain_length),
            current_angle_range=list(self.angle_range),
        )

        self.vessel.reset()
        self._wait_collision_clear()
        self.vessel.enableApiControl(True)
        self.vessel.armDisarm(True)
        if regen:
            self._generate_terrain()
            print(f"[Episode {self.episode_count}] Regenerated terrain")

        # Neutral controls
        self.vessel.setVesselControls("", VesselControls([0.0, 0.0], [0.5, 0.5]))

        # Clean up any leftover obstacles, then spawn fresh ones
        self._destroy_obstacles()
        self.current_obstacles = []
        self._spawn_obstacles()
        self._spawn_dynamic_obstacles()

        # Record collision timestamp so we ignore any stale/spawn collisions
        self._reset_collision_ts = self.vessel.simGetCollisionInfo().time_stamp

        # Reset waypoint navigation state
        self.current_waypoint_idx = 0
        start_state = self.vessel.getVesselState()
        self.prev_waypoint_x = start_state.kinematics_estimated.position.x_val
        self.prev_waypoint_y = start_state.kinematics_estimated.position.y_val

        # Reset tracking state
        self.prev_actions = np.zeros(2, dtype=np.float32)
        self.prev_distance_to_goal_x = 0.0
        self.prev_distance_to_goal_y = 0.0
        self.state["success"] = False
        self.state["collision"] = False
        self.episode_path_length = 0.0

        # Fill frame buffer with initial observation
        self.frame_buffer.clear()
        obs = self._get_obs()
        self.state["prev_position"] = self.state["position"].copy()
        self.initial_goal_distance = np.sqrt(
            self.state["distance_to_goal_x"]**2 +
            self.state["distance_to_goal_y"]**2
        )
        self._waypoint_start_distance = self.initial_goal_distance
        # Set distance milestone based on final waypoint (round down to nearest 10m)
        final_wp = self.waypoints[-1]
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]
        dist_to_final = np.sqrt((final_wp[0] - pos_x)**2 + (final_wp[1] - pos_y)**2)
        self.initial_final_goal_distance = dist_to_final
        self._next_distance_milestone = (int(dist_to_final) // 20) * 20
        self.episode_start_time = time.time()
        self.prev_reward_state = self._build_reward_state(goal_reached=False)
        self.episode_trajectory = [self.state["position"][:2].copy()]
        for _ in range(self.n_stack):
            self.frame_buffer.append(obs)
        diag_log(
            "env_reset_complete",
            episode_count=self.episode_count,
            requested_seed=requested_seed,
            regen=bool(regen),
            waypoint_count=len(self.waypoints),
            initial_goal_distance=float(self.initial_goal_distance),
            initial_final_goal_distance=float(self.initial_final_goal_distance),
            obstacle_count=len(self.current_obstacles),
        )
        return np.concatenate(list(self.frame_buffer)), {}

    def close(self):
        try:
            self.vessel.reset()
        except Exception:
            pass
