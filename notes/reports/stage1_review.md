# 阶段 1 验收复盘

## 1. 复盘目标

- 逐条核验阶段 1 的三项验收标准，确认能力已落地。
- 汇总"已掌握 / 待加强"清单，识别进入阶段 2 的风险点。
- 产出可追溯的能力证明（命令输出 + 日志片段）。

---

## 2. 验收范围

| 任务 | 主题 | 状态 |
|------|------|------|
| 4A | Service 最小闭环 | ✅ 已完成 |
| 4B | QoS 对比实验 | ✅ 已完成 |
| 4C | tf2 主动查询 | ✅ 已完成 |
| 4D | URDF 升级 xacro 参数化 | ✅ 已完成 |
| 4E | ros2_control 最小链路 | ✅ 已完成 |
| 4F | joint_trajectory_controller 联调 | ✅ 已完成 |
| 4G | forward_command_controller 对比 | ✅ 已完成 |
| 4H | 阶段 1 验收复盘（本文档） | 🔄 进行中 |

---

## 3. 完成情况总览

> 填写说明：用 1-2 句话描述每个任务的核心产出，不需要重复文档内容。

- **4A**：写了一个server和client服务，并且了解了topic/service/action的职责边界
- **4B**：了解了节点之间通信的QOS策略差异
- **4C**：学习了什么是TF树结构及其发布流程，实践操作掌握了如何查询TF2树
- **4D**：了解了从URDF升级到XACRO的意义，实践体验了通过XACRO参数化调整机器人结构的方法
- **4E**：通过JSB学习ROS2_CONTROL的最小链路调通方法，知道JSB的工作职责和流程
- **4F**：通过JTC学习了ROS2_CONTROL控制机器人的方法，体验了ACTION的功能
- **4G**：通过对比JTC和FCC了解了两种不同的控制器的控制策略，知道了机器人运动安全保障措施的职责应该有上层软件来把控，而不是由底层控制器来控制

---

## 4. 关键能力项验收

### 4.1 独立建包能力

> 验收标准：能独立创建并构建一个 ROS 2 package（Python 与 C++ 各至少一个）。

**验证命令：**
```bash
colcon build --packages-select ur3_stage1_review_py
ros2 run ur3_stage1_review_py demo_package_build
```

**实际输出（粘贴日志）：**
```
[INFO] [1775879310.811124482] [demo_package_build_node]: ============================================================
[INFO] [1775879310.811548739] [demo_package_build_node]:   验收点 1：独立建包能力演示
[INFO] [1775879310.811909364] [demo_package_build_node]: ============================================================
[INFO] [1775879310.812244524] [demo_package_build_node]:   包名       : ur3_stage1_review_py
[INFO] [1775879310.812561008] [demo_package_build_node]:   节点名     : demo_package_build_node
[INFO] [1775879310.812895513] [demo_package_build_node]:   命名空间   : /
[INFO] [1775879310.813231690] [demo_package_build_node]:   ROS 发行版 : jazzy
[INFO] [1775879310.813587221] [demo_package_build_node]: ------------------------------------------------------------
[INFO] [1775879310.813968172] [demo_package_build_node]:   依赖项可导入状态：
[INFO] [1775879310.814362549] [demo_package_build_node]:     [OK]  rclpy (rclpy)
[INFO] [1775879310.867590505] [demo_package_build_node]:     [OK]  sensor_msgs (sensor_msgs.msg)
[INFO] [1775879310.875483288] [demo_package_build_node]:     [OK]  trajectory_msgs (trajectory_msgs.msg)
[INFO] [1775879310.971971735] [demo_package_build_node]:     [OK]  control_msgs (control_msgs.action)
[INFO] [1775879310.972428260] [demo_package_build_node]: ------------------------------------------------------------
[INFO] [1775879311.004922914] [demo_package_build_node]:   已注册可执行入口 (console_scripts)：
[INFO] [1775879311.005344319] [demo_package_build_node]:     ros2 run ur3_stage1_review_py demo_action_rationale
[INFO] [1775879311.005908839] [demo_package_build_node]:     ros2 run ur3_stage1_review_py demo_description_reader
[INFO] [1775879311.006311163] [demo_package_build_node]:     ros2 run ur3_stage1_review_py demo_package_build
[INFO] [1775879311.006745027] [demo_package_build_node]: ============================================================
[INFO] [1775879311.007381039] [demo_package_build_node]:   [结论] 包结构正确，colcon 构建通过，节点可正常启动。
[INFO] [1775879311.007808456] [demo_package_build_node]: ============================================================
minzi@minzi-Lenovo-Legion-Y7000-2020H:~/ros2_lab$ cd ~/ros2_lab/workspaces/ws_tutorials/src
mkdir -p ur3_stage1_review_cpp/src
mkdir -p ur3_stage1_review_cpp/include/ur3_stage1_review_cpp
minzi@minzi-Lenovo-Legion-Y7000-2020H:~/ros2_lab/workspaces/ws_tutorials/src$ cd ~/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_stage1_review_cpp
source install/setup.bash
ros2 run ur3_stage1_review_cpp demo_package_build_cpp
Starting >>> ur3_stage1_review_cpp
Finished <<< ur3_stage1_review_cpp [4.74s]                     

Summary: 1 package finished [5.01s]
[INFO] [1775891254.581400586] [demo_package_build_cpp_node]: ============================================================
[INFO] [1775891254.581467905] [demo_package_build_cpp_node]:   验收点 1（C++）：独立建包能力演示
[INFO] [1775891254.581481768] [demo_package_build_cpp_node]: ============================================================
[INFO] [1775891254.581487155] [demo_package_build_cpp_node]:   包名   : ur3_stage1_review_cpp
[INFO] [1775891254.581491737] [demo_package_build_cpp_node]:   节点名 : demo_package_build_cpp_node
[INFO] [1775891254.581496119] [demo_package_build_cpp_node]:   命名空间: /
[INFO] [1775891254.581500258] [demo_package_build_cpp_node]:   [结论] C++ 包结构正确，colcon 构建通过，节点可正常启动。
[INFO] [1775891254.581505646] [demo_package_build_cpp_node]: ============================================================
```

**结论：**
- [X] Python 包：独立创建并构建通过
- [X] C++ 包：独立创建并构建通过
- 备注：

---

### 4.2 Action 适配性理解

> 验收标准：能清楚解释为何轨迹执行通常使用 Action 而非 Topic/Service。

**验证命令：**
```bash
ros2 run ur3_stage1_review_py demo_action_rationale
```

**实际输出（粘贴日志）：**
```
[INFO] [1775880092.737011488] [demo_action_rationale_node]: ============================================================
[INFO] [1775880092.737602256] [demo_action_rationale_node]:   验收点 2：Action 适配性分析 — 轨迹执行为何必须用 Action
[INFO] [1775880092.738069995] [demo_action_rationale_node]: ============================================================
[INFO] [1775880092.738571505] [demo_action_rationale_node]: 
[INFO] [1775880092.739049803] [demo_action_rationale_node]:   【方案 A】Topic（发布/订阅）
[INFO] [1775880092.739526957] [demo_action_rationale_node]:   ┌─ 发送方式 : publisher.publish(goal)
[INFO] [1775880092.740019428] [demo_action_rationale_node]:   ├─ 反馈机制 : 无，发完即忘
[INFO] [1775880092.740545376] [demo_action_rationale_node]:   ├─ 结果确认 : 无法知道轨迹是否执行完毕
[INFO] [1775880092.741056847] [demo_action_rationale_node]:   ├─ 中途取消 : 不支持
[INFO] [1775880092.741573955] [demo_action_rationale_node]:   └─ 结论     : ❌ 无法满足「等待完成 + 取消」需求
[INFO] [1775880092.742095979] [demo_action_rationale_node]: 
[INFO] [1775880092.742638897] [demo_action_rationale_node]:   【方案 B】Service（请求/响应）
[INFO] [1775880092.743056452] [demo_action_rationale_node]:   ┌─ 发送方式 : client.call(request)
[INFO] [1775880092.743385048] [demo_action_rationale_node]:   ├─ 反馈机制 : 仅有一次 response，无中间反馈
[INFO] [1775880092.743694647] [demo_action_rationale_node]:   ├─ 结果确认 : response 到达即为结果，但调用期间阻塞
[INFO] [1775880092.744004079] [demo_action_rationale_node]:   ├─ 中途取消 : 不支持
[INFO] [1775880092.744324170] [demo_action_rationale_node]:   ├─ 耗时场景 : 轨迹执行通常 1-30 秒，长期阻塞不可接受
[INFO] [1775880092.744637679] [demo_action_rationale_node]:   └─ 结论     : ❌ 无法做到「执行中持续反馈 + 随时取消」
[INFO] [1775880092.745020425] [demo_action_rationale_node]: 
[INFO] [1775880092.745352390] [demo_action_rationale_node]:   【方案 C】Action（目标/反馈/结果）✅
[INFO] [1775880092.745671155] [demo_action_rationale_node]:   ┌─ 发送方式 : client.send_goal_async(goal)
[INFO] [1775880092.745991767] [demo_action_rationale_node]:   ├─ 反馈机制 : 执行期间持续推送 feedback（当前关节位置等）
[INFO] [1775880092.746322698] [demo_action_rationale_node]:   ├─ 结果确认 : 轨迹完成后推送 result（成功/失败/取消）
[INFO] [1775880092.746651558] [demo_action_rationale_node]:   ├─ 中途取消 : client.cancel_goal_async() 可随时中止
[INFO] [1775880092.746982203] [demo_action_rationale_node]:   ├─ 非阻塞   : 异步设计，调用方不会被卡住
[INFO] [1775880092.747320453] [demo_action_rationale_node]:   └─ 结论     : ✅ 完全满足轨迹执行的三大需求
[INFO] [1775880092.747681414] [demo_action_rationale_node]: 
[INFO] [1775880092.748027622] [demo_action_rationale_node]:   ┌──────────────┬────────┬─────────┬────────┐
[INFO] [1775880092.748382704] [demo_action_rationale_node]:   │ 能力         │ Topic  │ Service │ Action │
[INFO] [1775880092.748727697] [demo_action_rationale_node]:   ├──────────────┼────────┼─────────┼────────┤
[INFO] [1775880092.749073774] [demo_action_rationale_node]:   │ 异步非阻塞   │  ✅   │   ❌   │  ✅   │
[INFO] [1775880092.749433027] [demo_action_rationale_node]:   │ 中间反馈     │  ❌   │   ❌   │  ✅   │
[INFO] [1775880092.749786606] [demo_action_rationale_node]:   │ 完成确认     │  ❌   │   ✅   │  ✅   │
[INFO] [1775880092.750141700] [demo_action_rationale_node]:   │ 随时取消     │  ❌   │   ❌   │  ✅   │
[INFO] [1775880092.750509504] [demo_action_rationale_node]:   └──────────────┴────────┴─────────┴────────┘
[INFO] [1775880092.751038207] [demo_action_rationale_node]: 
[INFO] [1775880092.751690126] [demo_action_rationale_node]:   [结论] FollowJointTrajectory 使用 Action 接口，
[INFO] [1775880092.752378238] [demo_action_rationale_node]:          正是因为它同时需要：异步执行 + 实时反馈 + 结果确认 + 可取消。
[INFO] [1775880092.753079036] [demo_action_rationale_node]: ============================================================
```

**口述要点（用自己的话填写）：**

1. Topic 的局限：发送后不会收到回复，不知道运行结果
2. Service 的局限：发送后可以收到回复，但不知道运行过程状态，也不能在过程中随时取消
3. Action 的优势：发送后可以收到运行状态FEEDBACK,也能知道最终结果，还能随时打断取消ACTION

**结论：**
- [X] 能清晰区分三种通信机制的适用场景
- 备注：

---

### 4.3 机械臂 description 包结构理解

> 验收标准：能阅读并说明一套机械臂 description 包的结构与参数入口。

**验证命令：**
```bash
ros2 run ur3_stage1_review_py demo_description_reader
```

**实际输出（粘贴日志）：**
```
[INFO] [1775880210.534113067] [demo_description_reader_node]: ============================================================
[INFO] [1775880210.534477399] [demo_description_reader_node]:   验收点 3：UR3 Description 包结构解析
[INFO] [1775880210.534770502] [demo_description_reader_node]: ============================================================
[INFO] [1775880210.535408634] [demo_description_reader_node]:   包名       : ur_description
[INFO] [1775880210.535731121] [demo_description_reader_node]:   share 路径 : /opt/ros/jazzy/share/ur_description
[INFO] [1775880210.536035806] [demo_description_reader_node]: 
[INFO] [1775880210.536337431] [demo_description_reader_node]:   关键子目录：
[INFO] [1775880210.546210799] [demo_description_reader_node]:     ✅ urdf/
[INFO] [1775880210.546656532] [demo_description_reader_node]:     ✅ meshes/
[INFO] [1775880210.546955906] [demo_description_reader_node]:     ✅ config/
[INFO] [1775880210.547296071] [demo_description_reader_node]:     ✅ launch/
[INFO] [1775880210.547787777] [demo_description_reader_node]: 
[INFO] [1775880210.548170880] [demo_description_reader_node]:   urdf/ 文件树：
[INFO] [1775880210.549522681] [demo_description_reader_node]:     urdf/inc/ur_common.xacro
[INFO] [1775880210.549984091] [demo_description_reader_node]:     urdf/inc/ur_joint_control.xacro
[INFO] [1775880210.550491265] [demo_description_reader_node]:     urdf/inc/ur_sensors.xacro
[INFO] [1775880210.550879783] [demo_description_reader_node]:     urdf/inc/ur_transmissions.xacro
[INFO] [1775880210.551326305] [demo_description_reader_node]:     urdf/ros2_control_mock_hardware.xacro
[INFO] [1775880210.551879941] [demo_description_reader_node]:     urdf/ur.urdf.xacro
[INFO] [1775880210.552367140] [demo_description_reader_node]:     urdf/ur_macro.xacro
[INFO] [1775880210.552771213] [demo_description_reader_node]:     urdf/ur_mocked.urdf.xacro
[INFO] [1775880210.553107222] [demo_description_reader_node]: 
[INFO] [1775880210.553808462] [demo_description_reader_node]:   [注] urdf/ 下未找到已展开的 .urdf 文件（仅有 .xacro），
[INFO] [1775880210.554275659] [demo_description_reader_node]:        xacro 主要参数入口如下：
[INFO] [1775880210.554998766] [demo_description_reader_node]:   常见 xacro 参数（在 ur.urdf.xacro 中通过 <xacro:arg> 声明）：
[INFO] [1775880210.555473658] [demo_description_reader_node]:     --xacro-args name:=<值>
[INFO] [1775880210.555932669] [demo_description_reader_node]:     --xacro-args prefix:=<值>
[INFO] [1775880210.556413334] [demo_description_reader_node]:     --xacro-args joint_limits_parameters_file:=<值>
[INFO] [1775880210.556881219] [demo_description_reader_node]:     --xacro-args kinematics_parameters_file:=<值>
[INFO] [1775880210.557365945] [demo_description_reader_node]:     --xacro-args physical_parameters_file:=<值>
[INFO] [1775880210.557826782] [demo_description_reader_node]:     --xacro-args visual_parameters_file:=<值>
[INFO] [1775880210.558295198] [demo_description_reader_node]:     --xacro-args safety_limits:=<值>
[INFO] [1775880210.558753367] [demo_description_reader_node]:     --xacro-args safety_pos_margin:=<值>
[INFO] [1775880210.559236904] [demo_description_reader_node]:     --xacro-args safety_k_position:=<值>
[INFO] [1775880210.559706711] [demo_description_reader_node]:   可用 xacro 文件：
[INFO] [1775880210.560173950] [demo_description_reader_node]:     ros2_control_mock_hardware.xacro
[INFO] [1775880210.560646656] [demo_description_reader_node]:     ur.urdf.xacro
[INFO] [1775880210.561118991] [demo_description_reader_node]:     ur_macro.xacro
[INFO] [1775880210.561600929] [demo_description_reader_node]:     ur_mocked.urdf.xacro
[INFO] [1775880210.562178033] [demo_description_reader_node]: ============================================================
[INFO] [1775880210.562794173] [demo_description_reader_node]:   [结论] ur_description 包包含 urdf/xacro + meshes + config + launch，
[INFO] [1775880210.563503446] [demo_description_reader_node]:          通过 xacro 参数（name/prefix/joint_limits_parameters_file 等）
[INFO] [1775880210.564170240] [demo_description_reader_node]:          可适配不同型号与场景，是机械臂系统集成的核心入口。
[INFO] [1775880210.564801628] [demo_description_reader_node]: ============================================================
```

**包结构说明（用自己的话填写）：**

| 文件 / 目录 | 职责 |
|-------------|------|
| `ur.urdf.xacro` |作为总入口，配置可变参数，构造YAML路径，并调用ur_macro.xacro 里的 xacro:ur_robot 宏，构建完整URDF |
| `ur_macro.xacro` |包含UR机械臂完整结构的模板，根据输入参数调用其他组件构建完整URDF |
| `inc/ur_common.xacro` |	link 几何形状、惯量、碰撞体、mesh 引用 |
| `inc/ur_joint_control.xacro` |关节的 ros2_control 接口声明（position/velocity command interface） |
| `config/ur3/joint_limits.yaml` |包含每个关节的限制（位置，速度，力矩） |
| `config/ur3/default_kinematics.yaml` |包含每个关节的 DH 参数（实际臂长和偏移） |

**结论：**
- [X] 能定位并解释 description 包的主要文件
- [X] 能说明修改关节限位应改哪个文件
- 备注：

---

## 5. 问题与风险

> 填写在学习过程中遇到的卡点、理解偏差或尚未解决的疑问。

| # | 问题描述 | 影响范围 | 改进动作 |
|---|----------|----------|----------|
| 1 |不理解QOS是什么 |不能按信息重要性调整信息传递策略 |已经学习QOS是什么 |
| 2 |不理解ROS2_CONTROL的框架内容是什么 |不能理解ROS2系统的真正意义 |已经学习ROS2_CONTROL框架的完整工作流程 |
| 3 |没有在SETUP.PY中配置LAUNCH文件 |不能启动LAUNCH文件 |已经让AI调整SETUP.PY文件 |

---

## 6. 经验总结

> 填写 3 条最有价值的收获，要求具体、可复用。

1. 使用CLAUDECODE的LEARN模式辅助自己学习ROS2
2. 自己配置CODEX的LEARN模式辅助学习ROS2
3. ros2_control框架包含硬件接口（分状态和指令两类）和控制器两种元素，控制器有很多种，主要功能就是作为高层指令和硬件接口之间的沟通渠道，读取硬件接口返回的状态信息，并向外部发布这些信息，也可以接受高层指令，向硬件接口写入控制指令，指挥机器人运动

---

## 7. 下一阶段进入条件

> 以下条件全部满足后，方可进入阶段 2。

- [x] 三项验收标准均已通过（4.1 / 4.2 / 4.3 全部打勾）
- [x] 风险表中的高优先级改进动作已处理或已制定计划
- [x] 阶段 2 首周目标已明确

---

## 8. 下一步计划

阶段 2：UR3 仿真控制入门（2 周）

目标：先在“非真机”模式下打通 UR3 控制链路。

UR 官方驱动支持几种模式，包括 fake hardware、真实硬件，以及从驱动角度等价对待的 URSim。官方也给了标准启动方式，例如通过 ur_control.launch.py 启动，并传入 ur_type 和 robot_ip。

你这一阶段的任务：

1）跑通官方 UR ROS 2 Driver
安装 ur_robot_driver
启动 fake hardware
看 joint states、controller 状态、TF 是否正常
2）理解控制器
scaled_joint_trajectory_controller
joint_trajectory_controller
speed scaling 的意义
teach pendant 上的速度限制如何影响外部控制
3）写自己的最小控制程序
用 Python 发一个简单关节轨迹
再用 C++ 写一版
学会订阅机器人状态、读取当前关节角

官方文档明确提到驱动包内带有示例节点，可作为你自己写 ROS 节点控制机器人的起点。

阶段验收：

你能在 fake hardware 模式下让 UR3 模型执行一段关节轨迹
你能解释“控制器、驱动、MoveIt”三者分别负责什么
