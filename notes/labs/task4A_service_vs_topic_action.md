# Task4A Service 与 Topic/Action 对比实验

## 1. 目标
- 跑通 `/ur3/set_mode` 的 Service Server + Client 最小闭环。
- 用真实调用结果说明 Topic / Service / Action 在控制场景中的边界。

## 2. 场景定义
- 场景：仅做“模式切换”（MANUAL <-> AUTO），不执行轨迹。
- 服务类型：`std_srvs/srv/SetBool`
- 语义约定：
  - `data=true` -> AUTO
  - `data=false` -> MANUAL
- 幂等策略：重复设置同一模式返回 success=true + already in target mode。

## 3. 接口设计
- 服务名：`/ur3/set_mode`
- 请求字段：`bool data`
- 响应字段：
  - `bool success`
  - `string message`
- 错误策略（本次最小实现）：保留失败分支接口，当前默认成功。

## 4. 实现步骤
1. 创建包：`ur3_mode_service_py`
2. 实现 `mode_service_server.py`（维护 `current_mode`）
3. 实现 `mode_service_client.py`（发起请求并打印返回）
4. 配置 `setup.py` console_scripts
5. `colcon build` + `ros2 run` 验证闭环

## 5. 关键命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_mode_service_py
source install/setup.bash
ros2 run ur3_mode_service_py mode_service_server
ros2 service call /ur3/set_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /ur3/set_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /ur3/set_mode std_srvs/srv/SetBool "{data: false}"
```

## 6. 运行现象与日志
- 首次切 AUTO：`success=True, message='mode switched to AUTO'`
- 重复切 AUTO：`success=True, message='already in AUTO'`
- 切回 MANUAL：`success=True, message='mode switched to MANUAL'`
- 异常/超时现象（如有）：`___`

## 7. 结论
- 为什么本场景用 Service：
  - `发布请求后需要立刻得到响应，并且本请求是一个短任务，不需要长时间跟踪`
- 为什么轨迹执行不适合用 Service：
  - `轨迹执行是长时间任务，过程中有状态变化，进度变化等信息，需要持续跟踪，同时还需要能够应付被突然取消等情况，并且需要返回最终结果；这些都不适合service完成，service只适合短时间的阻塞任务，没有过程量，只有请求和响应`（需提到：长时执行、feedback、cancel、result）
- 如果改用 Topic 会遇到什么问题：
  - `不知道服务端有没有接受到请求，客户端也不会得到直接的反馈信息`

## 8. 踩坑与排查
- 坑 1：`找不到launch文件`
  - 现象：`minzi@minzi-Lenovo-Legion-Y7000-2020H:~/ros2_lab/workspaces/ws_tutorials$ ros2 launch ur3_mode_service_py mode_service_demo.launch.py file 'mode_service_demo.launch.py' was not found in the share directory of package 'ur3_mode_service_py' which is at '/home/minzi/ros2_lab/workspaces/ws_tutorials/install/ur3_mode_service_py/share/ur3_mode_service_py'`
  - 根因：`setup.py文件中没有指定launch 文件，没有被安装到包的 share 目录里`
  - 修复：`在setup.py中补充launch文件的构建信息`
- 坑 2：`client.py文件和命令调用服务`
  - 现象：`client.py文件中直接写明了node.call(to_auto=True)，但是调用服务的指令ros2 service call /ur3/set_mode std_srvs/srv/SetBool "{data: false}"可以被正常接收`
  - 根因：`调用服务的指令和client.py本质上不是同一个客户端，可以理解称client.py是另一种调用指令的实现方式，两者并无直接关系`
  - 修复：`暂无`

## 9. 下一步
- 进入 4B QoS 实验前要先确认：
  - [x] 服务语义和幂等语义已稳定
  - [x] 至少 3 组调用日志已留档
  - [x] 已能口述 Service vs Action 选择依据
