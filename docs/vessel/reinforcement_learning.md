# Vessel Reinforcement Learning

## Introduction

The vessel reinforcement learning example in this repository is implemented under `PythonClient/reinforcement_learning/`.
It trains a local navigation policy for procedurally generated port channels: the policy receives vessel state, waypoint geometry, and LiDAR obstacle cues, then outputs continuous thrust and yaw commands.

The current training stack is:

- `PythonClient/reinforcement_learning/airgym/envs/vessel_env.py`
- `PythonClient/reinforcement_learning/crossq_vessel.py`
- `PythonClient/reinforcement_learning/eval_vessel.py`

This repository no longer uses the older `PythonClient/Vessel/Shipsim_gym.py` + SAC example described in legacy documentation.

## Known Issues

The current RL stack has two active issue threads that should be read together with this document:

- `docs/vessel/issues/issue_01_pcg_runtime_cleanup.md`
- `docs/vessel/issues/issue_02_rl_pipeline_semantics.md`

The first issue is about Unreal runtime scene cleanup and obstacle ownership.
The second issue is about Python-side training, evaluation, and curriculum semantics.

## Environment Overview (`PCGVesselEnv`)

`PCGVesselEnv` is a Gymnasium environment for procedural channel navigation with obstacle avoidance.
It uses the existing vessel PCG RPC APIs:

- `activateGeneration(False)`
- `generatePortTerrain(...)`
- `getGoal(initial_location, distance)`
- `simAddObstacle(...)`
- `getVesselState()`
- `getLidarData()`

### Task definition

- The vessel starts at its spawn point.
- At reset, the environment optionally regenerates a new procedural port terrain.
- For each generated section, the environment queries `getGoal(...)` and builds a waypoint chain.
- The active target is the current waypoint; when it is reached, the environment advances to the next waypoint.
- The episode succeeds only when the final waypoint is reached.

This means the RL policy is solving a local planning and control problem on top of the procedural channel generator, not generating the full global route directly.

### Environment Specifications

| Specification | Details |
| --- | --- |
| **Action Space** | `Box(2,)` - `[thrust, yaw_cmd]` |
| **Action Range** | `thrust: [0.0, 1.0]`, `yaw_cmd: [-1.0, 1.0]` |
| **Observation Space** | `Box(54,)` - waypoint geometry + vessel state + LiDAR |
| **Episode Length** | `800 / action_repeat` timesteps |
| **Success Condition** | Reach within 10 meters of the final waypoint |

### Observation Space

The observation vector has 54 elements:

```python
obs = [
    dx_prev_waypoint,
    dy_prev_waypoint,
    dx_current_waypoint,
    dy_current_waypoint,
    distance_to_current_waypoint,
    dx_next_waypoint,
    dy_next_waypoint,
    distance_to_next_waypoint,
    heading_error,
    sin_heading,
    cos_heading,
    linear_velocity_x,
    linear_velocity_y,
    linear_acceleration_x,
    linear_acceleration_y,
    angular_acceleration_z,
    prev_thrust,
    prev_yaw_cmd,
    lidar_sector_0,
    ...,
    lidar_sector_35,
]
```

The LiDAR observation is min-pooled into 36 sectors across 360 degrees.
Ground and vessel labels are filtered out so the observation focuses on obstacle distances.

### Reward Structure

| Condition | Reward |
| --- | --- |
| Progress toward current waypoint | `prev_distance - current_distance` |
| Time penalty | `-0.1` per step |
| Collision | `-100.0` and terminate |
| Final waypoint reached | `+500.0` and terminate |
| Timeout | truncate episode |

## Runtime Modes

The training and evaluation scripts support two simulator modes:

### 1. Attach to an existing Unreal Editor session

Recommended for local development and debugging in this repository.

```powershell
"E:\ProgramFile\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" "E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject"
cd E:\code\ASVSim\PythonClient\reinforcement_learning
python crossq_vessel.py --launch-sim none --ip 127.0.0.1 --num-waypoints 1
```

For routine RL work in `PortEnv`, rely on the project default map `GenerationTopDownTest`.
Do not pass an explicit map override on the `UnrealEditor.exe` command line unless you are intentionally testing another world.

When using `--launch-sim none`, the script does not start or kill the simulator.
If the Editor runtime becomes invalid, the script raises a clear error instead of attempting to restart `Blocks.exe`.

### 2. Launch a packaged simulator executable

Useful for batch runs when you have a packaged build available.

```powershell
cd E:\code\ASVSim\PythonClient\reinforcement_learning
python crossq_vessel.py --launch-sim exe --sim-path "E:\path\to\Blocks.exe"
```

In `exe` mode, the environment may restart the simulator automatically after a runtime failure.

## Training (`crossq_vessel.py`)

### Algorithm

Training uses `CrossQ` from `sb3-contrib` with `VecNormalize` and checkpointing.

### Basic Usage

Attach to a running Unreal Editor:

```powershell
python crossq_vessel.py `
  --launch-sim none `
  --ip 127.0.0.1 `
  --timesteps 2500000 `
  --terrain-regen 10 `
  --num-obstacles 4 `
  --num-dynamic-obstacles 0 `
  --num-waypoints 1 `
  --action-repeat 1 `
  --seed 43
```

Launch a packaged executable automatically:

```powershell
python crossq_vessel.py `
  --launch-sim exe `
  --sim-path "E:\path\to\Blocks.exe" `
  --sim-wait 10 `
  --timesteps 2500000
```

### Main Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `--ip` | `127.0.0.1` | Simulator RPC IP address |
| `--timesteps` | `2500000` | Total training timesteps |
| `--terrain-regen` | `10` | Regenerate PCG terrain every N episodes |
| `--num-obstacles` | `4` | Number of static obstacles per episode |
| `--num-dynamic-obstacles` | `0` | Number of moving obstacles per episode |
| `--num-waypoints` | `1` | Number of waypoint sections to navigate |
| `--launch-sim` | `none` | `none` attaches to an existing Editor/runtime, `exe` launches `--sim-path` |
| `--sim-path` | `Blocks/Blocks.exe` | Packaged simulator executable for `exe` mode |
| `--sim-wait` | `10` | Seconds to wait after launching the executable |
| `--sim-log` | `false` | Save launched simulator output to `logs/sim.log` |
| `--action-repeat` | `1` | Repeat each action N times |
| `--seed` | `43` | Random seed |
| `--wandb-key` | `None` | Enables Weights & Biases logging when set |

### Hyperparameters

```python
model = CrossQ(
    "MlpPolicy",
    env,
    learning_rate=0.0003,
    gamma=0.99,
    batch_size=256,
    buffer_size=500000,
    learning_starts=5000,
    train_freq=1,
    stats_window_size=10,
    policy_kwargs=dict(net_arch=[512, 512]),
)
```

### Output

Training artifacts are written to:

```text
logs/
├── training/
│   ├── models/
│   └── tb/
└── sim.log
```

## Evaluation (`eval_vessel.py`)

Evaluate while attached to a running Unreal Editor:

```powershell
python eval_vessel.py `
  --launch-sim none `
  --checkpoint logs/training/models/crossq_pcg_vessel_policy.zip `
  --episodes 20
```

Or evaluate with an auto-launched packaged executable:

```powershell
python eval_vessel.py `
  --launch-sim exe `
  --sim-path "E:\path\to\Blocks.exe" `
  --checkpoint logs/training/models/crossq_pcg_vessel_policy.zip
```

The evaluation script reports:

- success rate
- collision rate
- timeout rate
- average reward
- average final distance to the final waypoint

## Settings Requirements

Ensure `settings.json` contains a vessel with LiDAR enabled.
The RL environment assumes a vessel configuration compatible with:

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Vessel",
  "Vehicles": {
    "milliampere": {
      "VehicleType": "MilliAmpere",
      "HydroDynamics": {
        "hydrodynamics_engine": "FossenCurrent"
      },
      "PawnPath": "DefaultVessel",
      "AutoCreate": true,
      "RC": {
        "RemoteControlID": 0
      },
      "Sensors": {
        "lidar1": {
          "SensorType": 6,
          "Enabled": true,
          "NumberOfChannels": 8,
          "RotationsPerSecond": 1,
          "MeasurementsPerCycle": 450,
          "range": 100000,
          "X": 0,
          "Y": 0,
          "Z": -8.2,
          "Roll": 0,
          "Pitch": 0,
          "Yaw": 0,
          "VerticalFOVUpper": -2,
          "VerticalFOVLower": -10,
          "GenerateNoise": false,
          "DrawDebugPoints": false,
          "HorizontalFOVStart": -180,
          "HorizontalFOVEnd": 180
        },
        "Distance": {
          "SensorType": 5,
          "Enabled": true,
          "MaxDistance": 600,
          "DrawDebugPoints": false
        },
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        }
      }
    }
  }
}
```

## Python Dependencies

Install the Python dependencies from the repository:

```powershell
cd E:\code\ASVSim\PythonClient
pip install -r requirements.txt
```

At minimum, the RL example depends on:

- `gymnasium`
- `stable-baselines3`
- `sb3-contrib`
- `wandb`
- `numpy`
- `cosysairsim`

## Notes

- The procedural channel generator is reused as the source of waypoint targets.
- This example is meant to provide a runnable RL pipeline, not a final optimized reward or observation design.
- If `activateGeneration(False)` or `getGoal(...)` fails while attached to Unreal Editor, verify that the correct map is loaded and the PCG plugins are enabled in the project.
