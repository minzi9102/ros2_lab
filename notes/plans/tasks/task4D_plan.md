# 任务 4D 规划文档：URDF 升级 xacro 参数化（实践版）

## 1. 任务目标
- 将当前单体 `ur3_simplified.urdf` 升级为可参数覆盖的 xacro 结构。
- 建立“结构定义（macro）+ 参数入口（launch args）+ 运行验证（RViz/TF）”的最小闭环。
- 为 4E `ros2_control` 配置阶段提前准备稳定的 `robot_description` 生成路径。

## 2. 当前基线（来自仓库现状）
- 当前模型文件：`workspaces/ws_tutorials/src/ur3_joint_state_publisher_py/urdf/ur3_simplified.urdf`。
- 当前 launch 仍通过 `open(...).read()` 直接加载 URDF 文本，不支持参数覆盖。
- `setup.py` 仅安装 `urdf/*.urdf`，尚未安装 `*.xacro`。
- 阶段进度：4A/4B/4C 已完成，本任务只推进 4D，不并行 4E+。

## 3. 任务范围
- 包含：
  - xacro 文件骨架与宏拆分（entry + macro）；
  - 最小参数入口（几何参数 + 关节限位缩放）；
  - launch 接入 xacro 命令生成 `robot_description`；
  - A/B/C 三组参数化验证矩阵。
- 不包含：
  - 高精度惯量与碰撞模型重建；
  - 真实 UR 官方模型全量迁移；
  - ros2_control 标签注入（归属 4E）。

## 4. learn mode 分工
- 智能体负责（可直接完成）：
  - 脚手架文件创建；
  - launch 接线与安装项补齐；
  - 文档模板与验收口径整理；
  - 构建与最小运行路径检查。
- 人类负责（关键学习点）：
  - 参数语义设计（哪些参数外露、命名与默认值）；
  - 宏边界决策（拆几层、如何平衡可读性与复用）；
  - 关节限位策略（统一缩放还是按关节独立配置）。
- 约束：
  - 在人类完成关键决策前，智能体不替代性给出“最终参数体系答案”。

## 5. 实施步骤（3 天节奏）
1. Day 1：参数设计与文件布局
- 明确第一版参数集合：`link_2_length`、`tool0_offset_z`、`joint_limit_scale`。
- 落地 xacro 目录结构：
  - `ur3_simplified.urdf.xacro`（入口）
  - `ur3_simplified_macro.xacro`（结构定义）
- 输出“参数 -> 影响部件”映射表。

2. Day 2：接入 launch 与运行链路
- launch 从 xacro 动态生成 `robot_description`。
- 新增 3 个 launch 参数并透传到 xacro。
- 补齐安装与依赖（`*.xacro` + `xacro` 运行依赖）。

3. Day 3：实验验证与结论沉淀
- A 组：默认参数，行为应与当前模型一致。
- B 组：几何参数覆盖，验证模型形态/末端位置变化。
- C 组：限位参数覆盖，验证生成 URDF 中 limit 值变化。
- 将结论写入 `notes/concepts/urdf_xacro_parameterization.md`。

## 6. 实验矩阵（4D 最小可复现）

| 组别 | 输入参数 | 预期现象 | 证据 |
|---|---|---|---|
| A 默认 | `link_2_length=0.34, tool0_offset_z=0.12, joint_limit_scale=1.0` | 与旧 URDF 观感一致 | RViz + TF 日志 |
| B 几何覆盖 | 例如 `link_2_length=0.45` | 上臂/肘部位置明显变化 | RViz 对比截图或文字记录 |
| C 限位覆盖 | 例如 `joint_limit_scale=0.7` | limit 区间缩小 | `xacro -> urdf` 输出比对 |

## 7. 交付物
- 代码：
  - `urdf/*.xacro` 脚手架；
  - xacro 接入后的 launch；
  - 安装与依赖配置更新。
- 文档：
  - `notes/concepts/urdf_xacro_parameterization.md`（含命令、映射表、结论）。

## 8. 验收标准
- 能通过 launch 参数覆盖改变模型几何或限位行为。
- 能口述参数生效路径：`launch args -> xacro args -> robot_description -> robot_state_publisher/RViz`。
- 不破坏现有 `ur3_joint_state_publisher_py` 最小链路的可运行性。

## 9. 风险与回退
- 风险 1：参数过多导致学习焦点分散。
  - 回退：仅保留 2-3 个核心参数先闭环。
- 风险 2：宏拆分过深导致排错困难。
  - 回退：先单 macro 实现，再按需拆分。
- 风险 3：xacro 安装或依赖遗漏导致 launch 失败。
  - 回退：优先跑通 `xacro file.xacro` 命令，再恢复 launch。

## 10. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
