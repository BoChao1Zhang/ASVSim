# Issue 03: 当前 Environment 任务通过率的证据整理（不含改法）

## Summary

这份文档只整理当前已经确认的证据，不提出改法。

当前证据支持以下判断：

- 当前主要问题不是“训练/评估流程跑不起来”，而是“策略在预算内没有稳定完成 waypoint 通过”，本地 packaged probe 已能稳定完成训练与评估闭环，但评估结果为 5/5 timeout。[Probe]
- 当前任务本质上是**顺序 waypoint 跟踪与避障控制**，不是直接对最终 endpoint 做控制；只有最后一个 waypoint 被命中时，episode 才会以 `goal_reached` 结束。[Code]
- 当前日志把“控制时使用的当前 waypoint 距离”和“episode 结束时记录的最终 endpoint 距离”混在一起，会放大对 timeout 的误读。[Code]
- 当前控制问题不是单点 bug，更像由动作抽象、执行器动态、reward 结构和 episode budget 共同形成的结构性现象。[Code]
- 外部文献中，提高导航训练效率的常见方向主要包括：限制推进自由度、先验控制与学习残差混合、课程学习、任务分解、结构化状态表示；其中与 USV 场景最直接对应的证据是任务分解。[Literature]

## Scope

本文只包含四类内容：

- 本地代码事实
- 本地测试与 probe 产物
- 由代码常数直接推导出的现象解释
- 外部文献结论

本文不包含：

- 改动方案
- 参数调优建议
- 实施建议
- 路线图

## Evidence legend

- `[Code]`：直接由当前仓库源码支持
- `[Probe]`：直接由当前本地运行产物支持
- `[Derived]`：由当前代码常数或公式直接推导
- `[Literature]`：来自外部文献结论
- `[Limit]`：本轮未完全核实、需要谨慎解释

## Verified facts

### 1. 当前任务是顺序 waypoint 控制，而不是直接对最终 endpoint 控制

- 环境在 reset 后会按 section 逐个查询 waypoint，并将最后一个 waypoint 作为最终 goal 保存。`PythonClient/reinforcement_learning/airgym/envs/vessel_env.py:355-368` [Code]
- step 中用于控制、观测和 reward 的目标是当前激活 waypoint：`curr_wp = self.waypoints[self.current_waypoint_idx]`。`vessel_env.py:476-514`, `vessel_env.py:636-677` [Code]
- 只有最后一个 waypoint 被命中时才会 `terminated=True` 且 `goal_reached=True`；中间 waypoint 只会推进 `current_waypoint_idx`。`vessel_env.py:741-772` [Code]

结论：当前 RL 策略学到的是局部 waypoint 跟踪与避障控制，不是“直接规划到最终终点”。

置信度：高

### 2. timeout 是 episode 级预算耗尽，不会在 waypoint 切换时重置

- 默认 episode budget 由 `first_waypoint_max_timesteps + (num_waypoints - 1) * additional_waypoint_max_timesteps` 推导，默认值分别为 `1200` 和 `800`。`PythonClient/reinforcement_learning/config.py:45-49`, `config.py:200-239` [Code]
- 环境实际 step limit 使用 `resolve_env_step_limit(...)`，会再除以 `action_repeat`。`config.py:236-239`, `PythonClient/reinforcement_learning/eval_suite.py:367-372` [Code]
- timeout 条件是 `self.timestep >= self.max_timesteps`。`vessel_env.py:763-765` [Code]
- 预算语义已有单元测试覆盖。`PythonClient/reinforcement_learning/tests/test_episode_budget_semantics.py:6-26` [Code]

结论：当前 timeout 的含义是“在整个 episode 的决策预算内没有完成最后目标”，不是“某个 waypoint 的局部计时器到时”。

置信度：高

### 3. reset 后的船只会回到初始位姿，并清零动力学状态

- Unreal 侧会记录 pawn 的起始位置与起始旋转。`Unreal/Plugins/AirSim/Source/PawnSimApi.cpp:60-72` [Code]
- reset 时会将 pawn 传送回 `start_location` / `start_rotation`。`PawnSimApi.cpp:316-323` [Code]
- Vessel reset 会将 pose 以外的 twist、wrench、accelerations 清零，并重置物理线速度和角速度。`Unreal/Plugins/AirSim/Source/Vehicles/Vessel/VesselPawnSimApi.cpp:18-26`, `VesselPawnSimApi.cpp:189-203` [Code]
- Python env reset 后会重新置零动作缓存、重新计算初始距离，并重置 waypoint 索引。`vessel_env.py:894-994` [Code]

结论：当前“起步慢”不是因为 reset 时保留了前一回合的推进或转向速度。

置信度：高

### 4. packaged probe 已证明训练/评估闭环稳定，但没有出现成功到点

Probe 路径：

- `data/reinforcement_learning/runs/stability_probe_20260419_1630/config.yaml`
- `data/reinforcement_learning/runs/stability_probe_20260419_1630/eval_suite_results.csv`

已确认事实：

- probe 使用 `max_timesteps: 150`、`step_sleep: 0.05`、`action_repeat: 1`。`config.yaml:13-15` [Probe]
- `curriculum.enabled` 为 `false`，但 `curriculum.stages` 仍定义了 5 个 stage，eval suite 仍会用这些 stage 生成评估用例。`config.yaml:69-114`, `PythonClient/reinforcement_learning/eval_suite.py:177-206` [Code][Probe]
- 5 个 stage 的结果全部为 `timeout`，步数全部为 `150`，且 `collision_rate=0`、`sim_crash_rate=0`。`eval_suite_results.csv:1-6`, `eval_suite.py:258-281` [Code][Probe]
- 在当前 probe 目录下对 `fatal|error|exception|crash|sim_crash` 的日志检索没有命中。 [Probe]

结论：当前关键问题不是“模拟器不稳定”，而是“在预算内没通过任务”。

置信度：高

### 5. 当前日志口径混用了“当前 waypoint 距离”和“最终 endpoint 距离”

- step 期间，`distance_to_goal_x/y` 对应的是当前激活 waypoint。`vessel_env.py:636-641` [Code]
- reset 时记录的 `initial_goal_distance` 来自 episode 开始时的当前 waypoint 距离。`vessel_env.py:968-972` [Code]
- episode 结束时，`final_dist_x/final_dist_y` 使用的是 `self.waypoints[-1]`，也就是最终 waypoint。`vessel_env.py:820-842` [Code]
- `info["distance_to_goal_x"]` / `info["distance_to_goal_y"]` 也写入的是最终 waypoint 距离。`vessel_env.py:857-872` [Code]
- 训练回调把这个值记录成 `episode/final_distance_to_goal`。`PythonClient/reinforcement_learning/train.py:75-109` [Code]
- eval suite 同样按这个 `info` 字段汇总 `final_dist`。`PythonClient/reinforcement_learning/eval_suite.py:402-409` [Code]

结论：日志里“timeout 时离 goal 很远”不一定表示“离当前控制目标很远”，它可能只是表示“离最终 endpoint 仍远”。

置信度：高

### 6. 当前动作抽象是低层 `[thrust, yaw_cmd]` 控制

- 动作空间是 `[thrust, yaw_cmd]`，其中 `thrust ∈ [0, 1]`，`yaw_cmd ∈ [-1, 1]`。`vessel_env.py:135-141` [Code]
- 动作执行时，环境将同一 `thrust` 下发给两个推进器，并用 `yaw_cmd` 生成两个相反方向的角度偏置。`vessel_env.py:719-739` [Code]
- 底层执行器接收的角度控制信号是 `[0, 1]`，并按 `(u - 0.5) * 2 * max_rudder_angle` 映射为实际角度。`AirLib/include/vehicles/vessel/Thruster.hpp:49-54`, `Thruster.hpp:124-129` [Code]

结论：当前策略输出的是低层推进/转向命令，而不是“期望航向”或“期望速度”。

置信度：高

### 7. reward 明确同时奖励 progress 与 heading alignment，并含固定 step penalty

- reward 由 `progress`、`heading_align`、`obstacle_proximity`、`action_rate`、`cross_track`、`step_penalty`、`terminal` 组成。`PythonClient/reinforcement_learning/airgym/envs/reward.py:19-53` [Code]
- 当前默认权重为：`progress=1.0`、`heading_align=0.3`、`obstacle_proximity=0.5`、`action_rate=0.05`、`cross_track=0.2`、`step_penalty=0.1`。`PythonClient/reinforcement_learning/config.py:20-31` [Code]
- `step_penalty` 在 reward 实现中是固定的 `-1.0`，再乘以权重。`reward.py:34-50` [Code]

结论：reward 不是纯 progress reward，而是一个包含朝向、障碍、平滑项和时间惩罚的组合目标。

置信度：高

### 8. 执行器时间常数明确存在“推进慢于转向”的非对称性

- 通用 `ThrusterParams` 默认时间常数为：`thrust_signal_filter_tc = 2.0s`，`angle_signal_filter_tc = 0.5s`。`AirLib/include/vehicles/vessel/ThrusterParams.hpp:14-18` [Code]
- 一阶滤波离散更新形式为 `alpha = exp(-dt / tau)`。`AirLib/include/common/FirstOrderFilter.hpp:23-66` [Code]
- `Thruster` 内部对推力和角度分别套用了这两个滤波器。`Thruster.hpp:43-45`, `Thruster.hpp:65-85` [Code]

结论：底层执行器模型本身就让角度通道比推力通道更快进入稳态。

置信度：高

### 9. 当前有效最大舵角不能直接用通用默认值 30° 下结论

- 通用 `ThrusterParams` 默认 `max_rudder_angle = π / 6`，即 30°。`ThrusterParams.hpp:14-18` [Code]
- 但 `MilliAmpereParams` 会把两个 rudder 的 `max_rudder_angle` 覆盖为 `M_PI`。`AirLib/include/vehicles/vessel/parameters/MilliAmpereParams.hpp:62-65` [Code]

结论：任何基于“30° 默认舵角”的单值估算，在当前 vessel 类型确认为 `MilliAmpere` 时都不能直接当作最终事实。

置信度：高

## Derived implications from current code

### 10. 在当前时间常数下，转向通道比推进通道更快接近稳态

由一阶系统 90% 响应时间 `t90 ≈ tau * ln(10)` 可得：

- thrust: `tau=2.0s`，`t90 ≈ 4.61s`
- angle: `tau=0.5s`，`t90 ≈ 1.15s`

折算为决策步数：

- 当 `step_sleep = 0.25s`
  - thrust 约 `18.4` 步
  - angle 约 `4.6` 步
- 当 `step_sleep = 0.05s`
  - thrust 约 `92.1` 步
  - angle 约 `23.0` 步

以上直接由 `ThrusterParams.hpp` 与 `FirstOrderFilter.hpp` 的时间常数和更新公式推得。[Derived]

结论：在 RL step 视角下，当前系统天然更容易先形成明显转向，而不是快速形成稳定推进。

置信度：高

### 11. 仅从 reward 代数上看，“原地先对准”可能取得正回报

忽略 cross-track、动作变化和障碍项，只看：

- `reward ≈ progress + 0.3 * cos(heading_error) - 0.1`

则：

- `heading_error = 0°` 时，`0.3 * cos(e) - 0.1 = +0.20`
- `heading_error = 30°` 时，约 `+0.16`
- `heading_error = 45°` 时，约 `+0.11`
- `heading_error = 60°` 时，约 `+0.05`
- `heading_error = 90°` 时，约 `-0.10`

在障碍距离较大、cross-track 较小、动作变化也较小的情况下，即使 progress 很小，reward 也可能不为负。[Derived]

结论：reward 结构本身确实对“先把船头对准 waypoint”更友好，而不只奖励前向推进。

置信度：高

### 12. 当 `step_sleep` 变小时，固定 step penalty 会提高获得正回报所需的物理闭合速度

- 当前 step penalty 的绝对值为 `0.1 / step`。`reward.py:35-50`, `config.py:20-31` [Code]
- 若只考虑用 progress 抵消它，则需要 `Δd > 0.1 m / step`。[Derived]
- 折算为闭合速度：
  - `step_sleep = 0.25s` 时，约需 `0.40 m/s`
  - `step_sleep = 0.05s` 时，约需 `2.00 m/s`

结论：在 reward 口径不随 step 时间缩放时，更短的决策周期会让相同物理运动更难获得正的 progress-vs-time 净回报。[Derived]

置信度：高

## Probe observations

### 13. 当前 probe 的失败模式是 timeout-heavy，而不是 collision-heavy

- `easy / medium / hard / advanced / expert` 五个 stage 全部是 `timeout`。`eval_suite_results.csv:2-6` [Probe]
- 五个 stage 都没有 `goal_reached`。`eval_suite_results.csv:2-6` [Probe]
- 五个 stage 都没有 collision 记录；当前 probe 目录下也没有 crash 关键词命中。 [Probe]
- 不同 stage 的 `final_dist` 和 `path_length_ratio` 仍有变化，说明策略确实在运动，但没有在预算内完成任务。`eval_suite_results.csv:2-6` [Probe]

结论：当前问题更接近“推进效率不足 / 任务完成效率不足”，不是“环境完全不动”或“系统频繁崩溃”。

置信度：高

## External literature conclusions

### 14. 固定前向速度或将线速度限制为正值，在 mapless navigation 文献中很常见，但主要证据来自地面机器人

- Tai et al. (2017) 的 mapless navigation 连续控制设定中，线速度被固定为正常向值，策略主要学习转向控制。[Literature]
- Dotto de Moraes et al. (2023) 的 DDQN 导航实验中，线速度固定为常数，策略只选择角速度。[Literature]

结论：导航文献里确实存在“减少推进自由度、优先学习转向”的常见做法。

局限：这一组证据主要来自地面移动机器人，而不是 USV。[Limit]

置信度：中

### 15. 先验控制器与学习残差叠加，是外部文献里高频出现的效率提升模式

- Residual RL 的典型形式是 `u = π_H + π_θ`，即人工控制器给出基线动作，学习策略只输出残差。[Literature]
- Johannink et al. (2019) 的 Residual RL 工作明确将这种结构作为提高样本效率与最终性能的通用思路。[Literature]
- 相关导航类工作普遍把这种思路归类为“用稳定先验缩小 RL 需要学习的自由度”。[Literature]

结论：外部研究中，提高训练效率的常见方式不是完全从零学，而是“先验控制 + 学习修正”。

置信度：中高

### 16. 课程学习在 sparse-reward / collision-free navigation 中普遍有帮助

- Narvekar et al. (2020) 的综述明确指出，从简单任务逐步增加难度通常能够改善收敛速度和最终表现。[Literature]
- 导航领域的多篇工作也将 curriculum learning 作为解决 sparse reward 和困难探索问题的标准做法之一。[Literature]

结论：对于 timeout-heavy 的导航任务，训练样本的顺序本身就是关键变量。

置信度：高

### 17. 在 USV 场景里，将 goal approaching 与 obstacle avoidance 分开建模是有直接文献支持的

- Yang et al. (2023) 在文中明确写到，USV 路径规划可以拆成 `goal approaching` 与 `obstacle avoidance` 两个子目标，并指出单一优化策略难以同时实现两者。 [Literature]
- 用户提供的 2025 年 USV 文献也将 `navigation` 与 `obstacle avoidance` 视为可解耦训练的模块。[Literature]

结论：在 USV 场景中，“任务分解”不是例外，而是主流研究模式之一。

置信度：高

### 18. 结构化、预处理后的状态表示可能比原始传感器输入更有利于训练效率

- Mackay et al. (2022) 使用经过预处理的 dynamic obstacle velocity space，而不是直接输入原始感知数据，并将其作为提升导航训练效率的关键条件之一。[Literature]
- 更广泛的导航 RL 文献也常把“状态表达”而不只是“算法名称”视为收敛速度的重要因素。[Literature]

结论：训练效率问题不只取决于算法，也取决于 agent 实际看到的状态表示。

局限：这一点在当前报告中主要由少量代表性工作支撑，证据强度弱于任务分解与课程学习。[Limit]

置信度：中

## Limits

- 本文不讨论如何修改当前系统，只讨论当前证据能支持到哪里。 [Limit]
- 本文中的本地结论只对当前仓库代码和当前 probe 产物成立，不自动外推到其他分支或其他 packaged build。 [Limit]
- 一部分外部 PDF 在本轮检索中未能稳定抽出精确原文段落，因此文献部分保留为“结论级摘要”，不把未核实的长引文写入正文。 [Limit]
- 关于“当前最大实际舵角”的单值结论，本轮只保留参数覆盖事实，不保留未经运行时 vessel 类型再次核对的数值结论。 [Limit]

## Final conclusion

截至当前证据，最稳妥的结论是：

1. 当前主要问题不是仿真器 crash，而是 timeout-heavy 的任务完成不足。[Probe]
2. 当前任务定义本身是局部 waypoint 顺序控制，日志里又混用了当前 waypoint 与最终 endpoint 的距离，因此 timeout 的表面现象容易被误读。[Code]
3. 当前控制问题更像由低层动作抽象、推进/转向动态不对称、reward 组合目标和 episode budget 共同形成的结构性现象，而不是单个明显 bug。[Code][Derived]
4. 外部文献中，与“提高导航训练效率”最稳定相关的模式主要是：减少推进自由度、引入先验控制、课程学习、任务分解和结构化状态表示；其中与 USV 任务最直接对齐的是任务分解证据。[Literature]

本文到此为止，不进入改法与实施建议。