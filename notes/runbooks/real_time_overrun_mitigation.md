# 真机实时调度与 overrun 缓解 runbook

## 背景
- 8B/8C 真机只读 bringup 中，`controller_manager` 和 `UR_Client_Library` 均提示 FIFO 实时调度不可用。
- 当前系统基线：
  - 内核：`6.17.0-20-generic`，`PREEMPT_DYNAMIC`
  - 实时权限：`ulimit -r = 0`，`chrt -f 50 true` 失败
  - 用户组：`minzi` 不在 `realtime` 组
  - CPU governor：所有 CPU 为 `powersave`
  - lowlatency / rt-tests / cpufrequtils：未安装
- 结论：当前问题不是 UR driver 单点故障，而是系统实时权限、CPU governor 和内核配置共同不足。

## 风险解释
- FIFO 实时调度不可用时，`ros2_control_node` 无法稳定获得 500 Hz 控制循环所需的调度优先级。
- `powersave` governor 可能导致 CPU 频率响应偏保守，使偶发 overrun 更容易出现。
- generic 内核可用于只读 bringup 和极低风险学习实验，但不适合作为正式真机轨迹执行基线。

## 建议修复顺序

### 1. 安装基础工具与 lowlatency 内核
```bash
sudo apt update
sudo apt install linux-lowlatency rt-tests cpufrequtils
```

完成后重启，并在 GRUB 中选择 lowlatency 内核，或让系统默认进入 lowlatency：

```bash
uname -a
```

通过条件：输出中包含 `lowlatency`。

### 2. 配置 realtime 用户组与 limits
```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"

cat <<'EOF' | sudo tee /etc/security/limits.d/99-ros-realtime.conf
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
```

完成后必须退出登录并重新登录，或重启。

验证：

```bash
id
ulimit -r
ulimit -e
ulimit -l
chrt -f 50 true && echo "SCHED_FIFO OK"
```

通过条件：
- `id` 中包含 `realtime`
- `ulimit -r` 不再是 `0`
- `chrt -f 50 true` 成功

### 3. 将 CPU governor 切到 performance
临时切换：

```bash
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance | sudo tee "$cpu"
done
```

持久化配置：

```bash
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl enable --now cpufrequtils
```

验证：

```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c
```

通过条件：所有 CPU 均为 `performance`。

### 4. 延迟与 driver 验证
基础延迟测试：

```bash
sudo cyclictest -p 80 -m -n -i 1000 -l 10000
```

真机只读验证：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101
```

观察重点：
- `Could not enable FIFO RT scheduling policy` 应消失。
- `Your system/user seems not to be setup for FIFO scheduling` 应消失。
- overrun warning 应显著减少；若仍频繁出现，暂停动作任务并继续排查系统负载、BIOS 电源策略和后台进程。

## 8D 前准入建议
- 未完成本 runbook 前，最多允许继续做 8D dry-run 和状态门闩验证。
- 若必须在修复前做极低风险动作实验，需要满足：
  - reduced / 低速现场策略明确；
  - 单次动作幅度极小；
  - `speed_scaling > 0`；
  - `safety_mode` 可解释；
  - 现场急停可达；
  - overrun 不频繁刷屏。
- 正式轨迹执行前，建议完成 lowlatency、realtime limits、performance governor 三项。
