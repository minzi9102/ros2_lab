# 任务 5A 规划文档：跑通 UR ROS 2 Driver（fake hardware）

## 1. 任务目标
- 安装 `ur_robot_driver`，在 fake hardware 模式下完整启动 UR3 控制链路。
- 验证 joint states、controller 状态、TF 树三项指标均正常。
- 建立对"驱动层 → 控制器层 → 状态发布层"链路的直观认知，为 5B/5C 打基础。

## 2. 当前基线（来自仓库现状）
- 阶段 1 全部完成：具备 ros2_control 最小链路、joint_trajectory_controller 联调经验。
- 已有自建 xacro + ros2_control 标签的实践经验（4E/4F）。
- 尚未接触官方 UR 驱动包，UR3 fake hardware 启动链路未验证。

## 3. 任务范围（单功能约束）
- 包含：
  - `ur_robot_driver` 安装与依赖确认；
  - fake hardware 模式启动（`ur_control.launch.py` + `use_fake_hardware:=true`）；
  - joint states 话题验证、controller 状态验证、TF 树验证；
  - 启动链路关键参数说明（`ur_type`、`robot_ip`、`use_fake_hardware`）。
- 不包含：
  - 真实硬件或 URSim 接入；
  - 控制器深入对比（5B 范围）；
  - 自写控制节点（5C 范围）；
  - MoveIt 集成。

## 4. 核心概念框架（学习锚点）

| 层次 | 组件 | 职责 |
|---|---|---|
| 驱动层 | `ur_robot_driver` | 与硬件/仿真通信，暴露 hardware interface |
| 控制器层 | `ros2_control_node` + controllers | 读取 hardware interface，执行控制逻辑 |
| 状态发布层 | `joint_state_broadcaster` | 将 hardware interface 状态发布为 `/joint_states` |
| 描述层 | `robot_state_publisher` | 将 URDF + joint states 转换为 TF 树 |

fake hardware 模式：驱动层用软件模拟硬件接口，其余层与真机完全相同。

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 安装命令、依赖列表、launch 参数查询；
  - 验证命令清单与文档模板。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 观察 `ros2 control list_controllers` 输出，理解每个 controller 的状态含义；
  - 在 RViz 中打开 TF 树，确认 `base_link → tool0` 链路完整；
  - 对比 fake hardware 与自建 xacro 方案的差异，记录发现。

## 6. 实施步骤（2 天节奏）

### Day 1：安装与首次启动
1. 确认 ROS 2 Humble 环境，安装 `ur_robot_driver`：
   ```bash
   sudo apt install ros-humble-ur
   ```
2. 查阅官方 launch 文件入口：
   ```bash
   ros2 pkg prefix ur_robot_driver
   find $(ros2 pkg prefix ur_robot_driver) -name "ur_control.launch.py"
   ```
3. 以 fake hardware 模式启动 UR3：
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py \
     ur_type:=ur3 \
     robot_ip:=yyy.yyy.yyy.yyy \
     use_fake_hardware:=true \
     launch_rviz:=true
   ```
4. 验证 joint states：
   ```bash
   ros2 topic echo /joint_states --once
   ```

### Day 2：深度验证与文档沉淀
1. 验证 controller 状态：
   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```
2. 验证 TF 树：
   ```bash
   ros2 run tf2_tools view_frames
   # 或在 RViz 中添加 TF display
   ```
3. 记录以下问题的答案（写入交付文档）：
   - fake hardware 模式下哪些 controller 默认激活？
   - `/joint_states` 的 `position` 字段初始值是什么？
   - TF 树的根节点是什么，`tool0` 在哪一层？
4. 更新 `task5A_plan.md` 完成记录。

## 7. 关键命令（执行阶段使用）
```bash
# 安装驱动
sudo apt install ros-humble-ur

# 启动 fake hardware
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true launch_rviz:=true

# 验证三项指标
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 run tf2_tools view_frames
```

## 8. 交付物
- 文档：`notes/labs/task5A_ur_driver_fake_hardware.md`
  - 包含：启动步骤、三项验证截图/输出、关键参数说明、发现与疑问。

## 9. 验收标准
- `ros2 topic echo /joint_states` 能输出 6 个关节的 position/velocity/effort。
- `ros2 control list_controllers` 显示至少 `joint_state_broadcaster` 和一个轨迹控制器为 `active`。
- RViz 中 TF 树从 `base_link` 到 `tool0` 链路完整，无断链警告。
- 能口头解释 fake hardware 模式与真机模式的本质区别。

## 10. 风险与回退
- 风险 1：`ur_robot_driver` 版本与 ROS 2 Humble 不兼容。
  - 回退：检查 `apt-cache policy ros-humble-ur`，确认版本；必要时从源码构建。
- 风险 2：`robot_ip` 参数在 fake hardware 模式下仍被校验导致启动失败。
  - 回退：查阅官方文档确认 fake hardware 模式是否需要真实 IP；尝试 `robot_ip:=192.168.56.101`（URSim 默认地址）。
- 风险 3：RViz 启动后 TF 树显示断链。
  - 回退：先确认 `robot_state_publisher` 是否运行，再检查 URDF 是否正确加载。

## 11. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
