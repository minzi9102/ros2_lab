# ws_ur3e_controller HOME/READY GUI 仿真优先执行方案

本文档定义 `ws_ur3e_controller` 的第一版 HOME/READY 图形界面执行方案。

当前阶段不直接进入真机 GUI 执行，而是先让 GUI 在两种仿真环境下完成完整功能闭环：

- `fake hardware + MoveIt/RViz`
- `URSim + ur_robot_driver + MoveIt/RViz`

真机执行保留为后续独立阶段。

## 1. 阶段目标

第一步只追求一个小而完整的仿真 GUI 闭环：

- 启动一个独立 GUI 小窗口。
- GUI 连接现有 service：`/ur3e_named_motion_controller/execute_named_target`。
- 点击 `HOME` 按钮后发送 `home` 执行请求。
- 点击 `READY` 按钮后发送 `ready` 执行请求。
- 界面显示 service 连接状态、请求状态和最近一次响应。
- 后端返回失败时，界面直接显示 `status` 和 `message`，不自动重试、不自动恢复。
- 在 fake hardware 和 URSim 两套仿真环境中完成 `READY -> HOME` execute 验收。

## 2. YAGNI / KISS 原则

本阶段遵守以下约束：

- 不新增控制协议。
- 不修改 `ExecuteNamedTarget.srv`。
- 不修改 `named_motion_controller_node.cpp` 的执行门闩逻辑。
- 不修改 `ur3e_named_targets.yaml` 中 `home` / `ready` 的点位。
- 不新增第三个命名目标。
- 不做目标编辑、目标管理或 catalog 可视化。
- 不做 waypoint 队列、连续控制、自动重试、自动恢复或 Action cancel。
- 不引入 Web 服务、数据库、rqt 插件或复杂前端框架。
- GUI 只做薄客户端，规划、执行和安全检查继续由现有 C++ 控制器负责。

## 3. 技术方案

新增独立 Python ROS 2 GUI 包：

```text
ur3e_named_motion_gui_py
```

技术栈：

- `rclpy`
- `python_qt_binding`
- Qt Widgets
- `ur3e_controller_msgs/srv/ExecuteNamedTarget`

新增入口：

```text
ros2 launch ur3e_named_motion_gui_py sim_named_motion_gui.launch.py
```

选择独立 Qt 小面板的原因：

- 本机已有 ROS 2 Python 和 Qt 绑定路径。
- 相比 rqt 插件，独立小窗口脚手架更少，调试路径更短。
- 相比 Web 面板，不需要引入额外 bridge、server 或浏览器通信层。

## 4. GUI 行为

界面只包含第一版必需控件：

- `HOME` 按钮
- `READY` 按钮
- service 状态显示
- 请求状态显示
- 最近一次 service 响应显示

按钮请求内容固定为：

```text
HOME:
target_name: home
execute: true
human_confirmation: ""

READY:
target_name: ready
execute: true
human_confirmation: ""
```

第一阶段按仿真 execute 设计：

- GUI 默认不携带真机确认 token。
- 点击 `HOME` 或 `READY` 后直接发送 execute 请求。
- 不额外弹出确认框。
- 不在 GUI 中复制 Dashboard、controller、speed scaling 等检查。

请求期间：

- 两个运动按钮临时禁用。
- 状态显示为请求中。
- service 返回后恢复按钮状态。

service 不可用时：

- 界面显示 service 未连接。
- 按钮禁用，或点击后显示 service 不可用。
- 不启动控制器、不启动 MoveIt、不尝试自动修复环境。

响应处理：

- `status == "executed"` 且 `executed == true` 时显示成功。
- 其他状态均显示失败或拒绝，并展示后端返回的 `status` 和 `message`。
- 不根据失败原因自动重试或执行恢复命令。

## 5. 两种仿真环境

### 5.1 fake hardware + MoveIt/RViz

启动完整 fake hardware 仿真、MoveIt、RViz，并允许执行：

```bash
ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  runtime_mode:=sim \
  execute:=true \
  use_mock_hardware:=true \
  launch_rviz:=true
```

另开终端启动 GUI：

```bash
ros2 launch ur3e_named_motion_gui_py sim_named_motion_gui.launch.py
```

验收：

- GUI 显示 service connected。
- 点击 `READY` 返回 `status=executed`。
- 点击 `HOME` 返回 `status=executed`。
- 两次响应均包含 final-target gate passed。
- RViz 中能观察到机械臂从 `home -> ready -> home`。

### 5.2 URSim + ur_robot_driver + MoveIt/RViz

URSim 侧前置条件：

- URSim 已启动 UR3e 仿真。
- ROS PC 能访问 URSim IP。
- URSim 已准备 driver 所需的 External Control 环境。

启动同一个 bringup，但关闭 fake hardware 并指定 URSim IP：

```bash
ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  runtime_mode:=sim \
  execute:=true \
  use_mock_hardware:=false \
  robot_ip:=<URSim IP> \
  launch_rviz:=true
```

另开终端启动 GUI：

```bash
ros2 launch ur3e_named_motion_gui_py sim_named_motion_gui.launch.py
```

验收：

- GUI 显示 service connected。
- 点击 `READY` 返回 `status=executed`。
- 点击 `HOME` 返回 `status=executed`。
- RViz 与 URSim 中的机器人状态一致。

URSim 第一阶段仍使用 `runtime_mode:=sim`，不触发真机确认 token 与 real gate。

## 6. 推荐文件与接口

新增内容：

```text
src/ur3e_named_motion_gui_py/
  package.xml
  setup.py
  setup.cfg
  resource/ur3e_named_motion_gui_py
  ur3e_named_motion_gui_py/
    __init__.py
    main.py
    request_logic.py
  launch/
    sim_named_motion_gui.launch.py
  test/
    test_request_logic.py
```

ROS 参数：

- `service_name`
  - 默认：`/ur3e_named_motion_controller/execute_named_target`
- `human_confirmation`
  - 默认：空字符串
- `poll_period_ms`
  - 默认：`200`

第一阶段不增加 GUI 内的仿真/真机复杂模式切换。通过 controller bringup 的 launch 参数和运行说明区分 fake hardware 与 URSim。

## 7. 测试计划

单元测试覆盖请求构造和响应解释：

- `HOME` 请求字段正确：
  - `target_name == "home"`
  - `execute == true`
  - `human_confirmation == ""`
- `READY` 请求字段正确：
  - `target_name == "ready"`
  - `execute == true`
  - `human_confirmation == ""`
- service 返回 `executed` 时，GUI 状态解释为成功。
- service 返回 `rejected_real_gate` 时，GUI 状态解释为失败并保留后端 message。
- service 返回 `planning_failed` 时，GUI 状态解释为失败并保留后端 message。
- service 返回 `execution_failed` 时，GUI 状态解释为失败并保留后端 message。

手动替代验证：

- 用 mock service 返回固定成功响应，验证 `HOME` / `READY` 按钮可触发请求。
- 用 mock service 返回固定失败响应，验证界面显示后端 `status` 和 `message`。
- 在 service 不存在时启动 GUI，验证按钮不可用或清晰显示 service 未连接。

集成验收：

1. fake hardware 下使用 GUI 完成 `READY -> HOME` execute。
2. URSim 下使用 GUI 完成 `READY -> HOME` execute。
3. 继续运行现有 `ur3e_named_motion_controller_cpp` 测试，确认 controller 行为未被 GUI 改动影响。

## 8. Learn Mode 分工

适合智能体直接完成：

- Python 包脚手架。
- launch 文件。
- service client 胶水代码。
- 请求构造函数。
- 响应状态解释函数。
- 轻量测试。
- 使用说明文档补充。

适合人类重点参与：

- 启动并观察 URSim 环境。
- 确认 URSim IP 与 External Control 环境。
- 在 RViz 和 URSim 中确认 `READY -> HOME` 运动符合预期。
- 后续真机阶段重新确认一键按钮是否符合现场安全流程。

## 9. 暂不做的事项

第一阶段明确不做：

- 真机执行。
- GUI 内仿真 / 真机复杂模式切换。
- 点位新增、删除、编辑。
- catalog 文件读取和可视化。
- Dashboard 恢复命令。
- controller 启停控制。
- MoveIt、URSim 或真机 bringup 自动启动。
- 操作日志持久化。
- 权限系统。
- 多机器人支持。
- 多目标队列。
- 急停功能。

急停和物理安全链路应继续依赖机器人现场已有安全机制，而不是由这个 GUI 第一版承担。

## 10. 推荐下一步

1. 构建 `ur3e_named_motion_gui_py`。
2. 运行 `test_request_logic.py` 验证请求和响应解释。
3. 在 fake hardware 中手动验收 `READY -> HOME`。
4. 在 URSim 中手动验收 `READY -> HOME`。
5. 两套仿真都稳定后，再单独规划真机 GUI 阶段。
