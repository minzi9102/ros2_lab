# ros2_lab

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
│   ├── labs/                   # 实验记录（Task 4A/4B/4C/4G 等）
│   ├── plans/                  # 学习计划与任务拆解
│   │   ├── stage1_next_steps_learning_plan.md
│   │   └── tasks/
│   │       ├── task4A_plan.md
│   │       ├── task4B_plan.md
│   │       ├── task4C_plan.md
│   │       ├── task4D_plan.md
│   │       ├── task4E_plan.md
│   │       ├── task4F_plan.md
│   │       ├── task4G_plan.md
│   │       └── task4H_plan.md
│   ├── reports/                # 阶段性复盘
│   ├── runbooks/               # 调试操作手册
│   └── ur3_control_chain_task3.md
├── scripts/                    # 辅助脚本
├── workspaces/
│   └── ws_tutorials/           # 当前主要 ROS 2 工作区
│       ├── src/                # ROS 2 包源码
│       │   ├── ur3_follow_joint_trajectory_client_py/
│       │   ├── ur3_joint_state_publisher_py/
│       │   ├── ur3_mode_service_py/
│       │   ├── ur3_qos_lab_py/
│       │   └── ur3_state_monitor_cpp/
│       ├── build/              # colcon 构建产物
│       ├── install/            # colcon 安装产物
│       └── log/                # 工作区构建/测试日志
└── log/                        # 仓库级 colcon 日志
```

说明：`build/`、`install/`、`log/` 为构建与运行过程中生成的目录，会随实验推进持续变化。
