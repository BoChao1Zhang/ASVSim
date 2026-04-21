# ASVSim 操作文档

## 1. 文档范围

本文档基于当前仓库 `E:\code\ASVSim` 的实际结构整理，目标是回答三件事：

1. 如何在当前项目上完成环境配置。
2. 如何启动并运行 Vessel 强化学习训练与评估。
3. 如何在当前项目中采集图像、状态和 ROS 数据。

本文档以 **Windows + Unreal Engine 5.7 + PortEnv 工程** 为主线，所有命令、路径和脚本都尽量对齐当前仓库现状，而不是沿用旧版公开文档中的示例路径。

## 2. 当前项目的关键目录

| 路径 | 用途 |
| --- | --- |
| `E:\code\ASVSim` | 仓库根目录 |
| `E:\code\ASVSim\build.cmd` | Windows 下原生库与 Unreal 插件构建入口 |
| `E:\code\ASVSim\Unreal\Environments\PortEnv\Blocks.uproject` | 当前主用 Unreal 工程 |
| `E:\code\ASVSim\Unreal\Environments\PortEnv\Config\DefaultEngine.ini` | 默认启动地图配置，当前是 `GenerationTopDownTest` |
| `E:\code\ASVSim\Unreal\Environments\PortEnv\package.bat` | 打包 `PortEnv` 为 `Blocks.exe` |
| `E:\code\ASVSim\PythonClient\requirements.txt` | Python 依赖列表 |
| `E:\code\ASVSim\PythonClient\reinforcement_learning\crossq_vessel.py` | 当前 Vessel RL 训练入口 |
| `E:\code\ASVSim\PythonClient\reinforcement_learning\eval_vessel.py` | 当前 Vessel RL 评估入口 |
| `E:\code\ASVSim\PythonClient\reinforcement_learning\airgym\envs\vessel_env.py` | 当前 RL 环境定义 `PCGVesselEnv` |
| `E:\code\ASVSim\PythonClient\Vessel\data_generation\dataset_generation.py` | Python 图像数据采集脚本 |
| `E:\code\ASVSim\PythonClient\Vessel\data_generation\bounding_boxes.py` | 分割图转检测框的后处理脚本 |
| `E:\code\ASVSim\ros\python_ws\src\airsimros\launch` | ROS/rosbag 数据采集入口 |

## 3. 推荐环境

### 3.1 Unreal / Visual Studio

当前仓库建议使用以下组合：

- Unreal Engine：`E:\ProgramFile\UE_5.7`
- Visual Studio：`C:\Program Files\Microsoft Visual Studio\18\Community`
- 推荐开发者命令入口：

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
```

原因如下：

- `build.cmd` 会检查 `VisualStudioVersion`，要求在 Visual Studio 原生命令环境中执行。
- 对 VS 18.x / UE 5.7，当前仓库明确提示优先使用 **MSVC 14.44**。
- 当前 `PortEnv` 工程的 `EngineAssociation` 是 `5.7`。

### 3.2 Python

建议单独创建一个 Python 虚拟环境。当前仓库内的 Python 客户端包为 `cosysairsim`，强化学习脚本依赖 `gymnasium`、`stable-baselines3`、`sb3-contrib`、`wandb` 等库。

建议步骤：

```powershell
Set-Location E:\code\ASVSim
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r PythonClient\requirements.txt
pip install -e PythonClient
pip install pandas
```

说明：

- `pip install -r PythonClient\requirements.txt` 用于安装 RL 与常用 API 脚本依赖。
- `pip install -e PythonClient` 让 `cosysairsim` 以可编辑模式安装，方便你在仓库内修改 Python API 后立即生效。
- `bounding_boxes.py` 额外依赖 `pandas`，但 `PythonClient\requirements.txt` 没有包含它，因此需要单独安装。

## 4. Windows 下的完整构建流程

### 4.1 构建原生 AirSim / ASVSim 插件

在 **VS 开发者命令行** 中执行：

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
cd /d E:\code\ASVSim
build.cmd --Debug
```

可选构建模式：

```bat
build.cmd --Release
build.cmd --RelWithDebInfo
```

执行后，`build.cmd` 会做几件事：

1. 下载并构建 `rpclib`。
2. 下载 `Eigen`。
3. 编译 `AirSim.sln`。
4. 将产物复制到 `Unreal\Plugins\AirSim\Source\AirLib`，供 Unreal 工程使用。

### 4.2 刷新 PortEnv 工程

原生构建完成后，进入 `PortEnv`：

```bat
cd /d E:\code\ASVSim\Unreal\Environments\PortEnv
update_from_git.bat
```

`update_from_git.bat` 的作用：

- 使用 `Blocks.uproject` 中的 `AdditionalPluginDirectories` 指向仓库根目录的 `Unreal\Plugins`。
- 重新生成工程文件。
- 不再创建本地插件镜像。

### 4.3 打开 Unreal 工程

你可以使用以下任一方式：

```powershell
Set-Location E:\code\ASVSim\Unreal\Environments\PortEnv
.\Blocks.uproject
```

或者：

```powershell
Set-Location E:\code\ASVSim\Unreal\Environments\PortEnv
& "E:\ProgramFile\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe" .\Blocks.uproject
```

当前工程默认启动地图是：

```text
/Game/FlyingCPP/Maps/GenerationTopDownTest
```

在编辑器中点击 `Play` 后，AirSim RPC 服务才会真正进入可连接状态。

### 4.4 首次验证：确认 Vessel API 能连通

启动编辑器并进入 `Play` 后，在已激活的 Python 虚拟环境中执行：

```powershell
Set-Location E:\code\ASVSim\PythonClient\Vessel
python .\hello_vessel.py
```

如果运行正常，你应看到：

- `confirmConnection()` 成功。
- `enableApiControl(True)` 成功。
- 终端打印一份 `getVesselState()` 返回的状态。
- 船体会执行一次简单控制动作。

这一步用于确认：

- Unreal 工程已经正常加载 AirSim 插件。
- RPC 端口默认仍为 `41451`。
- Python API 可以控制当前 Vessel Pawn。

## 5. `settings.json` 的放置与加载

### 5.1 推荐位置

Windows 下推荐把配置文件放在：

```text
C:\Users\<你的用户名>\Documents\AirSim\settings.json
```

AirSim 的配置搜索顺序中，这个位置是最稳定的默认方案。

### 5.2 当前项目推荐的基础配置

下面给出一个适合当前 `PortEnv` / Vessel / 传感器 / 录制的基础模板。该模板综合了当前仓库的 `docs\settings_example.json`、ROS 启动文件默认命名、以及 Vessel 使用方式。

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Vessel",
  "ApiServerPort": 41451,
  "ClockSpeed": 1.0,
  "ViewMode": "SpringArmChase",
  "RecordUIVisible": true,
  "LogMessagesVisible": true,
  "InitialInstanceSegmentation": true,
  "SubWindows": [
    { "WindowID": 0, "CameraName": "frontcamera", "ImageType": 0, "VehicleName": "airsimvehicle", "Visible": true },
    { "WindowID": 1, "CameraName": "frontcamera", "ImageType": 5, "VehicleName": "airsimvehicle", "Visible": false },
    { "WindowID": 2, "CameraName": "frontcamera", "ImageType": 3, "VehicleName": "airsimvehicle", "Visible": false }
  ],
  "Recording": {
    "RecordOnMove": false,
    "RecordInterval": 0.1,
    "Enabled": false,
    "Folder": "E:/ASVSimData/recordings",
    "Cameras": [
      { "CameraName": "frontcamera", "ImageType": 0, "PixelsAsFloat": false, "VehicleName": "airsimvehicle", "Compress": true },
      { "CameraName": "frontcamera", "ImageType": 5, "PixelsAsFloat": false, "VehicleName": "airsimvehicle", "Compress": true },
      { "CameraName": "frontcamera", "ImageType": 3, "PixelsAsFloat": true,  "VehicleName": "airsimvehicle", "Compress": false }
    ]
  },
  "CameraDefaults": {
    "CaptureSettings": [
      { "ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90, "MotionBlurAmount": 0 },
      { "ImageType": 3, "Width": 1280, "Height": 720, "FOV_Degrees": 90 },
      { "ImageType": 5, "Width": 1280, "Height": 720, "FOV_Degrees": 90, "MotionBlurAmount": 0 }
    ]
  },
  "Vehicles": {
    "airsimvehicle": {
      "VehicleType": "MilliAmpere",
      "PawnPath": "DefaultVessel",
      "AutoCreate": true,
      "HydroDynamics": {
        "hydrodynamics_engine": "FossenCurrent"
      },
      "RC": {
        "RemoteControlID": 0
      },
      "Cameras": {
        "frontcamera": {
          "X": 0.25, "Y": 0.0, "Z": -0.3,
          "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0
        },
        "backcamera": {
          "X": -0.25, "Y": 0.0, "Z": -0.3,
          "Roll": 0.0, "Pitch": 0.0, "Yaw": 180.0
        }
      },
      "Sensors": {
        "imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "lidar": {
          "SensorType": 6,
          "Enabled": true,
          "External": false,
          "ExternalLocal": true,
          "NumberOfChannels": 1,
          "Range": 120,
          "RotationsPerSecond": 10,
          "MeasurementsPerCycle": 3600,
          "X": 0.0,
          "Y": 0.0,
          "Z": -0.55,
          "Roll": 0.0,
          "Pitch": 0.0,
          "Yaw": 0.0,
          "VerticalFOVUpper": 0.0,
          "VerticalFOVLower": 0.0,
          "HorizontalFOVStart": 0.0,
          "HorizontalFOVEnd": 360.0,
          "DrawDebugPoints": false,
          "GenerateNoise": false
        },
        "gpulidar": {
          "SensorType": 8,
          "Enabled": true,
          "NumberOfChannels": 32,
          "Range": 50,
          "Resolution": 1024,
          "RotationsPerSecond": 10,
          "MeasurementsPerCycle": 512,
          "X": 0.0,
          "Y": 0.0,
          "Z": -0.3,
          "Roll": 0.0,
          "Pitch": 0.0,
          "Yaw": 0.0
        },
        "echo": {
          "SensorType": 7,
          "Enabled": true,
          "RunParallel": true,
          "X": 0.0,
          "Y": 0.0,
          "Z": -0.55,
          "Roll": 0.0,
          "Pitch": 0.0,
          "Yaw": 0.0,
          "MeasurementFrequency": 5,
          "NumberOfTraces": 1000,
          "SenseActive": true,
          "SensePassive": true,
          "PassiveRadius": 10
        }
      }
    }
  }
}
```

### 5.3 关于 RL LiDAR 配置的重要说明

当前仓库里的 `PCGVesselEnv` 会把 LiDAR 点云直接重排为：

```python
(36, 100)
```

也就是每个观测步默认假设有 **3600 个二维距离点**。因此：

- 旧文档中那种 `MeasurementsPerCycle = 450` 的配置已经 **不匹配当前 RL 环境代码**。
- 当前 RL 训练最简单的配置方式，是把 `lidar` 设为：

```json
"NumberOfChannels": 1,
"MeasurementsPerCycle": 3600
```

- 当前代码按上游 code contract 处理 RL state 里的传感器采样，不再提供 Python 侧 `lidar_noise_sigma` / `heading_noise_sigma` 这类额外观测噪声开关。
- 也就是说，state 里的 heading 与 LiDAR 读数直接使用运行时传感器输出，前提是你的 `settings.json` 已满足这里的 3600 点 LiDAR 契约。

如果你的 LiDAR 配置与此不一致，`vessel_env.py` 在重排点云时就可能报错。

## 6. 启动 ASVSim 的两种方式

### 6.1 方式 A：编辑器内启动

适合：

- 调试物理、蓝图、地图和传感器。
- 手工验证 API。
- 采集少量交互式数据。

步骤：

1. 打开 `Blocks.uproject`。
2. 确认地图为默认 PortEnv 地图 `GenerationTopDownTest`。
3. 点击 `Play`。
4. 用 Python 客户端连接 RPC。

### 6.2 方式 B：打包成 `Blocks.exe` 后启动

适合：

- 批量训练 RL。
- 长时间无人值守运行。
- 避免编辑器额外负担。

打包命令：

```bat
cd /d E:\code\ASVSim\Unreal\Environments\PortEnv
package.bat E:\ASVSimBuilds E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles 5.7
```

打包成功后，默认可执行文件路径通常为：

```text
E:\ASVSimBuilds\Blocks\Blocks.exe
```

说明：

- `package.bat` 会先构建编辑器目标，再调用 Unreal 的 `RunUAT BuildCookRun`。
- `PortEnv\Config\DefaultGame.ini` 已经包含 `GenerationTopDownTest` 与 `FlyingExampleMap` 的烹饪配置，其中 RL 默认地图是 `GenerationTopDownTest`。
- 当前 RL 训练脚本默认会自己启动一个模拟器进程，因此 **训练更推荐使用打包后的 `Blocks.exe`**。

## 7. 启动 RL 训练

## 7.1 当前真实训练入口

当前仓库的 Vessel RL 训练入口不是旧文档里的 `PythonClient/Vessel/Shipsim_gym.py`，而是：

```text
E:\code\ASVSim\PythonClient\reinforcement_learning\crossq_vessel.py
```

它调用的环境类是：

```text
E:\code\ASVSim\PythonClient\reinforcement_learning\airgym\envs\vessel_env.py
```

算法是：

```text
sb3_contrib.CrossQ
```

当前控制接口也已经回对齐到上游 code contract：

- `thrust` 范围为 `[0.0, 0.7]`
- `rudder_signal` 范围为 `[0.4406, 0.5594]`
- `vessel_env.py` 会把第二维动作直接作为 rudder signal，下发为 `VesselControls([0, thrust], [0, rudder_signal])`

### 7.2 训练前提

训练前必须满足以下条件：

1. `build.cmd` 已成功执行，Unreal 插件已同步到 `PortEnv`。
2. `settings.json` 已配置为 `SimMode = Vessel`。
3. `PortEnv` 已成功打包为 `Blocks.exe`，或者你自己改造了训练脚本的启动方式。
4. `Blocks.uproject` 中的 `PCG` 和 `PCGGeometryScriptInterop` 插件保持启用。
5. `lidar` 传感器配置满足当前 `PCGVesselEnv` 的 3600 点假设。

### 7.3 标准训练命令

推荐从脚本所在目录运行，这样日志输出目录最清晰：

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\crossq_vessel.py `
  --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe `
  --sim-wait 15 `
  --timesteps 2500000 `
  --terrain-regen 10 `
  --num-obstacles 4 `
  --num-dynamic-obstacles 0 `
  --num-waypoints 1 `
  --action-repeat 1 `
  --seed 43
```

### 7.4 关键参数解释

| 参数 | 含义 |
| --- | --- |
| `--sim-path` | 模拟器可执行文件路径。默认脚本写的是 `Blocks/Blocks.exe`，当前仓库一般需要改成你实际打包出来的绝对路径。 |
| `--sim-wait` | 启动模拟器后等待多少秒再连接 RPC。首次启动或低性能机器建议适当增大。 |
| `--timesteps` | 总训练步数。 |
| `--terrain-regen` | 每隔多少个 episode 重建一次程序化港口地形。 |
| `--num-obstacles` | 每回合静态障碍物数量。 |
| `--num-dynamic-obstacles` | 每回合动态障碍物数量。 |
| `--num-waypoints` | 航点数量，当前脚本说明为 `1` 或 `2`。 |
| `--action-repeat` | 每次动作重复施加的次数。增大后，单回合最大步数会变成 `800 // action_repeat`。 |
| `--seed` | 随机种子。 |
| `--wandb-key` | 可选。传入后会启用 Weights & Biases 记录。 |

### 7.5 输出目录

训练输出默认写到当前工作目录下：

```text
logs/
├── sim.log                      # 仅在 --sim-log 时生成
└── training/
    ├── models/
    │   ├── crossq_pcg_vessel_25000_steps.zip
    │   ├── crossq_pcg_vessel_50000_steps.zip
    │   └── crossq_pcg_vessel_policy.zip
    └── tb/
```

说明：

- 检查点每 `25000` 步保存一次。
- 最终模型默认保存为 `crossq_pcg_vessel_policy.zip`。
- TensorBoard 日志保存在 `logs/training/tb/`。

### 7.6 训练过程中的运行逻辑

当前 `PCGVesselEnv` 的主要行为包括：

- 初始化时连接 `VesselClient`。
- 调用 `activateGeneration(False)` 激活程序化生成。
- 在 `reset()` 时按策略生成港口地形、获取航点、清理旧障碍并重新生成障碍。
- 在 `step()` 中读取 `getVesselState()` 和 `getLidarData()`。
- 若模拟器异常，会尝试重启 `Blocks.exe` 并继续训练。

这也意味着：

- **训练脚本天然更适合配合打包后的 `Blocks.exe` 使用。**
- 如果只开着编辑器而没有可重启的 `Blocks.exe`，自动恢复逻辑不会完整生效。

### 7.7 一个适合先跑通的最小训练命令

第一次建议先缩小规模，确认流程能跑：

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\crossq_vessel.py `
  --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe `
  --sim-wait 20 `
  --timesteps 50000 `
  --terrain-regen 0 `
  --num-obstacles 2 `
  --num-dynamic-obstacles 0 `
  --num-waypoints 1 `
  --action-repeat 1 `
  --seed 43
```

这个组合的意义：

- `--terrain-regen 0`：固定地形，便于复现和调试。
- `--timesteps 50000`：先验证路径、日志和 checkpoint 是否正常。
- `--num-obstacles 2`：降低物理与训练复杂度。

## 8. 评估训练结果

评估脚本入口：

```text
E:\code\ASVSim\PythonClient\reinforcement_learning\eval_vessel.py
```

示例命令：

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\eval_vessel.py `
  --checkpoint .\logs\training\models\crossq_pcg_vessel_policy.zip `
  --episodes 20 `
  --terrain-regen 0 `
  --num-obstacles 2 `
  --num-dynamic-obstacles 0 `
  --num-waypoints 1 `
  --action-repeat 1 `
  --seed 43
```

评估脚本会在终端打印：

- 每个 episode 的 `reward`
- `goal_reached / collision / timeout`
- 航点到达数
- 最终距离
- 汇总统计

注意：

- `eval_vessel.py` 本身不会在启动时主动拉起模拟器进程。
- 如果你想稳定使用自动恢复能力，依然建议让 `sim_path` 指向可执行的 `Blocks.exe`。

## 9. 数据采集方案

当前项目中至少有三类常用数据采集方式：

1. AirSim 内置 `Recording`
2. `dataset_generation.py` 图像数据脚本
3. ROS / rosbag 采集

### 9.1 方案 A：AirSim 内置 Recording

这是最省事的方式，适合：

- 采集相机图像
- 同步记录位姿、速度等基础运动学状态
- 不想自己写采集脚本

#### 配置方式

在 `settings.json` 中配置 `Recording` 段，例如：

```json
"Recording": {
  "RecordOnMove": false,
  "RecordInterval": 0.1,
  "Enabled": false,
  "Folder": "E:/ASVSimData/recordings",
  "Cameras": [
    { "CameraName": "frontcamera", "ImageType": 0, "PixelsAsFloat": false, "VehicleName": "airsimvehicle", "Compress": true },
    { "CameraName": "frontcamera", "ImageType": 5, "PixelsAsFloat": false, "VehicleName": "airsimvehicle", "Compress": true },
    { "CameraName": "frontcamera", "ImageType": 3, "PixelsAsFloat": true,  "VehicleName": "airsimvehicle", "Compress": false }
  ]
}
```

#### 启动方式

你可以用两种方式开始录制：

1. 在模拟器界面中按 `R` 键。
2. 通过 Python API 调用：

```python
import cosysairsim as airsim

client = airsim.VesselClient()
client.confirmConnection()
client.startRecording()

# ... 执行控制或等待 ...

client.stopRecording()
```

#### 输出内容

默认会生成：

```text
<Recording Folder>\<timestamp>\
├── airsim_rec.txt
└── images\
```

其中：

- `airsim_rec.txt` 是 tab 分隔的轨迹与状态记录。
- `images\` 保存录制的相机图像。

限制：

- 内置录制只会保存 `Recording` 里指定的内容。
- 如果你需要更多字段，例如自定义传感器、额外标签或更特殊的组织格式，需要自己写脚本或改动录制代码。

### 9.2 方案 B：Python 图像采集脚本

脚本位置：

```text
E:\code\ASVSim\PythonClient\Vessel\data_generation\dataset_generation.py
```

它的功能是：

- 连接 `VesselClient`
- 自动创建本次采集目录
- 抓取 `Scene`、`Segmentation`、`DepthVis`
- 保存 `segmentation_colormap_list.csv`

#### 重要前提

当前脚本在 `generate_dataset()` 中把相机写死成了：

```python
ImageRequest("4", ...)
```

这意味着你需要二选一：

1. 保持脚本默认不动，并依赖车辆默认数值相机 `"4"`。
2. 把脚本中的 `"4"` 改成你已经在 `settings.json` 里配置好的相机名，例如 `"frontcamera"`。

如果你已经全面切换到命名相机，推荐直接把三处 `"4"` 改成 `"frontcamera"`，避免配置和脚本不一致。

#### 运行方式

建议从 `PythonClient` 目录运行，因为脚本里的默认输出目录是相对路径：

```powershell
Set-Location E:\code\ASVSim\PythonClient
python .\Vessel\data_generation\dataset_generation.py
```

#### 默认输出目录

脚本默认把数据写到：

```text
E:\code\ASVSim\PythonClient\Vessel\data_generation\dataset\<timestamp>\
```

目录结构如下：

```text
dataset\<timestamp>\
├── depth\
├── rgb\
├── segmentation\
└── segmentation_colormap_list.csv
```

#### 默认运行参数

在脚本 `__main__` 里当前写死了：

```python
dataset_path = "Vessel/data_generation/dataset"
num_images = 10
max_depth = 255
```

也就是说，若你想调整采集规模或输出位置，最简单的方法是直接修改这三个变量。

#### 采集时的行为

脚本会：

- 持续给船体一个固定推力和固定舵角。
- 每轮抓取 3 张图：RGB、分割、深度。
- 如果检测到碰撞，就调用 `reset()` 并重新给控制。

适用场景：

- 需要同步的 RGB / segmentation / depth 图像。
- 需要后续做检测框生成或分割训练。

### 9.3 方案 C：分割图后处理为检测框

脚本位置：

```text
E:\code\ASVSim\PythonClient\Vessel\data_generation\bounding_boxes.py
```

它的作用是：

- 读取 `segmentation` 图像。
- 结合 `segmentation_colormap_list.csv` 查对象颜色。
- 生成 XML 检测框。
- 可选叠加距离信息。

#### 使用注意

当前 `bounding_boxes.py` 的 `__main__` 同样是硬编码路径和对象前缀的：

```python
dataset_path = "Vessel/data_generation/dataset/2025_04_10_13_14_00/"
```

因此，实际使用时通常要改两类内容：

1. `dataset_path` 改成你本次采集目录。
2. `get_all_instances_of_object(...)` 里的对象前缀改成你真正想提取的目标名称。

#### 运行方式

```powershell
Set-Location E:\code\ASVSim\PythonClient
python .\Vessel\data_generation\bounding_boxes.py
```

运行后会在数据集目录下生成：

```text
bounding_boxes\
```

每张图对应一个 `.xml`。

### 9.4 方案 D：ROS / rosbag 数据采集

如果你需要完整 ROS 话题流、TF、传感器点云和可重放数据集，当前仓库提供了 Python ROS 包装层。

关键目录：

```text
E:\code\ASVSim\ros\python_ws
```

适用场景：

- 需要与 ROS 算法栈对接。
- 需要录制和回放统一 rosbag。
- 需要传感器数据和轨迹严格同步。

#### 说明

当前仓库的 ROS 文档是按 **Ubuntu / ROS Noetic** 描述的，因此这部分通常运行在：

- 原生 Linux
- WSL2
- 远程 Ubuntu 机器

而不是纯 Windows PowerShell。

#### 常见流程

1. 构建工作区：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

或者直接使用仓库自带工作区：

```bash
cd /mnt/e/code/ASVSim/ros/python_ws
catkin_make
source devel/setup.bash
```

2. 在线发布 AirSim 话题：

```bash
roslaunch airsimros airsim_publish.launch
```

3. 仅录制路线：

```bash
roslaunch airsimros airsim_record_route.launch
```

4. 回放路线并记录所有传感器：

```bash
roslaunch airsimros airsim_replay_route_record_sensors.launch
```

默认 launch 文件已经定义了以下命名约定：

- Vehicle：`airsimvehicle`
- IMU：`imu`
- Echo：`echo`
- LiDAR：`lidar`
- GPU LiDAR：`gpulidar`
- Camera：`frontcamera`、`backcamera`

因此如果你希望 ROS 配置与 `settings.json` 无缝对齐，建议也采用这些名字。

## 10. 常见问题与排查

### 10.1 `Blocks.exe` 不存在，RL 无法启动

原因：

- 当前仓库默认主工程是 `PortEnv` 编辑器工程。
- `crossq_vessel.py` 默认写死了 `Blocks/Blocks.exe`，但仓库里并不会天然存在这个文件。

解决：

1. 先执行 `package.bat` 打包。
2. 在训练命令中显式传入：

```powershell
--sim-path E:\ASVSimBuilds\Blocks\Blocks.exe
```

### 10.2 训练脚本能启动，但很快在 LiDAR 维度报错

原因：

- 当前 `PCGVesselEnv` 代码会把点云强制重排成 `(36, 100)`。
- 旧文档里的 `450` 点 LiDAR 配置已经过时。

解决：

- 把 RL 使用的 `lidar` 设为 **总点数 3600**，例如：

```json
"NumberOfChannels": 1,
"MeasurementsPerCycle": 3600
```

- 当前代码按上游 code contract 处理 RL state 里的传感器采样，不再提供 Python 侧 `lidar_noise_sigma` / `heading_noise_sigma` 这类额外观测噪声开关。
- 也就是说，state 里的 heading 与 LiDAR 读数直接使用运行时传感器输出，前提是你的 `settings.json` 已满足这里的 3600 点 LiDAR 契约。

### 10.3 `activateGeneration()` 或 `getGoal()` 失败

重点检查：

1. 当前启动的是不是 `PortEnv` 对应的工程或打包产物，而不是别的 Blocks 环境。
2. `Blocks.uproject` 里的 `PCG` 和 `PCGGeometryScriptInterop` 插件有没有被关闭。
3. 是否真的进入了 `Play` 状态。
4. `settings.json` 是否处于 `SimMode = Vessel`。

### 10.4 Python 采集脚本抓不到图像

重点检查：

1. 是否已经进入 `Play`。
2. `dataset_generation.py` 使用的相机名是否与你配置一致。
3. 分辨率或图像类型是否在 `CameraDefaults` 中正确配置。
4. 工作目录是否正确，避免相对路径写到错误位置。

### 10.5 Recording 有输出，但字段不够

这是正常现象。内置 `Recording` 的设计目标是轻量采集，不是全量传感器导出。

解决路径有两个：

1. 用 Python / ROS API 自己拉传感器数据并保存。
2. 直接修改仓库里的录制代码，扩展 `airsim_rec.txt` 输出字段。

## 11. 推荐的实际操作顺序

如果你的目标是最快把当前项目跑通，推荐按下面顺序执行：

1. 构建原生插件：

```bat
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
cd /d E:\code\ASVSim
build.cmd --Debug
```

2. 刷新 Unreal 工程：

```bat
cd /d E:\code\ASVSim\Unreal\Environments\PortEnv
update_from_git.bat
```

3. 写好 `Documents\AirSim\settings.json`。

4. 打开 `Blocks.uproject`，保持项目默认地图 `GenerationTopDownTest`，然后点击 `Play`。

5. 跑通 API 烟测：

```powershell
Set-Location E:\code\ASVSim\PythonClient\Vessel
python .\hello_vessel.py
```

6. 若要训练 RL，先打包：

```bat
cd /d E:\code\ASVSim\Unreal\Environments\PortEnv
package.bat E:\ASVSimBuilds E:\ProgramFile\UE_5.7\Engine\Build\BatchFiles 5.7
```

7. 启动最小训练：

```powershell
Set-Location E:\code\ASVSim\PythonClient\reinforcement_learning
python .\crossq_vessel.py --sim-path E:\ASVSimBuilds\Blocks\Blocks.exe --timesteps 50000 --terrain-regen 0 --num-obstacles 2
```

8. 若要采集图像数据：

```powershell
Set-Location E:\code\ASVSim\PythonClient
python .\Vessel\data_generation\dataset_generation.py
```

9. 若要生成检测框：

```powershell
Set-Location E:\code\ASVSim\PythonClient
python .\Vessel\data_generation\bounding_boxes.py
```

## 12. 相关源码入口

如果你要继续扩展本文档或修改行为，最值得优先看的文件是：

- `build.cmd`
- `Unreal\Environments\PortEnv\Blocks.uproject`
- `Unreal\Environments\PortEnv\Config\DefaultEngine.ini`
- `Unreal\Environments\PortEnv\Config\DefaultGame.ini`
- `PythonClient\requirements.txt`
- `PythonClient\reinforcement_learning\crossq_vessel.py`
- `PythonClient\reinforcement_learning\eval_vessel.py`
- `PythonClient\reinforcement_learning\airgym\envs\vessel_env.py`
- `PythonClient\Vessel\data_generation\dataset_generation.py`
- `PythonClient\Vessel\data_generation\bounding_boxes.py`
- `ros\python_ws\src\airsimros\launch\airsim_publish.launch`
- `ros\python_ws\src\airsimros\launch\airsim_record_route.launch`
- `ros\python_ws\src\airsimros\launch\airsim_replay_route_record_sensors.launch`
