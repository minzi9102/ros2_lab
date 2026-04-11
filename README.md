# ros2_lab

ROS 2 Jazzy 机械臂开发学习仓库，以 UR3 为目标平台，按阶段推进。

当前仓库目录结构（按现状更新）：

```text
~/ros2_lab/
├── AGENTS.md                   # 协作约定（learn mode）
├── README.md
├── archive/                    # 归档内容
├── docs/                       # 资料文档
├── downloads/                  # 临时下载内容
├── notes/                      # 学习笔记与计划
│   ├── concepts/               # 概念笔记
│   │   ├── ros2_control_minimal_chain.md
│   │   └── urdf_xacro_parameterization.md
│   ├── labs/                   # 实验记录
│   │   ├── task3_ur3_control_chain.md
│   │   ├── task4A_service_vs_topic_action.md
│   │   ├── task4B_qos_experiment.md
│   │   ├── task4C_tf2_lookup.md
│   │   ├── task4F_joint_trajectory_controller.md
│   │   └── task4G_controller_comparison.md
│   ├── plans/                  # 学习计划与任务拆解
│   │   ├── archive/            # 阶段总规划（已归档）
│   │   │   ├── stage1_learning_plan.md
│   │   │   └── stage2_learning_plan.md
│   │   └── tasks/              # 子任务规划文档
│   │       ├── task4A_plan.md ~ task4H_plan.md   # 阶段 1（已完成）
│   │       ├── task5A_plan.md  # 跑通 UR ROS 2 Driver（fake hardware）
│   │       ├── task5B_plan.md  # 理解控制器体系
│   │       └── task5C_plan.md  # 最小控制程序（Python + C++）
│   └── reports/                # 阶段性复盘
│       └── stage1_review.md
├── scripts/                    # 辅助脚本
├── workspaces/
│   └── ws_tutorials/           # 当前主要 ROS 2 工作区
│       ├── src/                # ROS 2 包源码
│       │   ├── ur3_follow_joint_trajectory_client_py/   # 4E/4F：轨迹 Action Client
│       │   ├── ur3_forward_command_controller_lab_py/   # 4G：forward_command_controller 对比
│       │   ├── ur3_joint_state_publisher_py/            # 基础：关节状态发布
│       │   ├── ur3_joint_trajectory_controller_lab_py/  # 4F：joint_trajectory_controller 联调
│       │   ├── ur3_mode_service_py/                     # 4A：Service 最小闭环
│       │   ├── ur3_qos_lab_py/                          # 4B：QoS 对比实验
│       │   ├── ur3_ros2_control_lab_py/                 # 4E：ros2_control 最小链路
│       │   ├── ur3_stage1_review_cpp/                   # 4H：阶段 1 验收（C++）
│       │   ├── ur3_stage1_review_py/                    # 4H：阶段 1 验收（Python）
│       │   ├── ur3_state_monitor_cpp/                   # 基础：状态监控（C++）
│       │   └── ur3_tf_lookup_py/                        # 4C：tf2 主动查询
│       ├── build/              # colcon 构建产物
│       ├── install/            # colcon 安装产物
│       └── log/                # 工作区构建/测试日志
└── log/                        # 仓库级 colcon 日志
```

## 学习进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 阶段 1 | ROS 2 基础能力（Task 4A–4H） | 已完成 |
| 阶段 2 | UR3 仿真控制入门（Task 5A–5C） | 进行中 |

说明：`build/`、`install/`、`log/` 为构建与运行过程中生成的目录，会随实验推进持续变化。
