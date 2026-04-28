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

如果重启后 `cpufrequtils` 已启用但实际 governor 仍回到 `powersave`，检查是否被桌面电源策略覆盖：

```bash
powerprofilesctl get
powerprofilesctl list
systemctl is-active power-profiles-daemon
```

若 `power-profiles-daemon` 处于 `balanced` 或 `power-saver`，可在真机实验前临时切到 performance：

```bash
powerprofilesctl set performance
```

验证：

```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c
cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c
```

通过条件：
- 非 `intel_pstate` 环境：所有 CPU governor 均为 `performance`；
- `intel_pstate` active 环境：`powerprofilesctl get` 为 `performance`，且 `energy_performance_preference` 为 `performance`。此时 `scaling_governor` 仍可能显示 `powersave`，不要把这一项单独当作失败。

### 4. 延迟与 driver 验证
基础延迟测试：

```bash
sudo cyclictest -p 80 -m -i 1000 -l 10000 -q
```

> 注：部分 Ubuntu 24.04 / rt-tests 版本的 `cyclictest` 不支持旧命令里的 `-n` 参数。若输出 `invalid option -- 'n'`，使用上面的无 `-n` 版本。

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
- 期望看到 `Successful set up FIFO RT scheduling policy with priority 50` 与 `SCHED_FIFO OK`。
- overrun warning 应显著减少；若仍频繁出现，暂停动作任务并继续排查系统负载、BIOS 电源策略和后台进程。

## 2026-04-28 验证记录
- 内核：`6.8.0-110-lowlatency`，已进入 lowlatency。
- realtime 权限：`id` 已包含 `realtime`；`ulimit -r = 99`；`ulimit -l = unlimited`；`chrt -f 50 true` 成功。
- 工具：`cyclictest`、`cpufreq-info`、`chrt` 可用。
- `cyclictest -p 80 -m -i 1000 -l 10000 -q`：`Min 2 us / Avg 6 us / Max 51 us`；同时提示 `/dev/cpu_dma_latency` 权限不足，说明本轮未成功锁住电源管理延迟约束。
- driver 只读验证：FIFO warning 已消失，日志出现 `Successful set up FIFO RT scheduling policy with priority 50` 与 `SCHED_FIFO OK, priority 99`。
- 未完全通过项：当前 `powerprofilesctl get` 为 `balanced`，实际 12 个 CPU governor 仍为 `powersave`；`cpufrequtils` 虽为 enabled/active 且 `/etc/default/cpufrequtils` 已写 `GOVERNOR="performance"`，但被更高层电源策略覆盖。
- 补充验证：执行 `powerprofilesctl set performance` 后，`powerprofilesctl get` 为 `performance`，12 个 CPU 的 `energy_performance_preference` 均为 `performance`，当前频率约 `3.00 GHz`。由于系统使用 `intel_pstate active`，`scaling_governor` 仍显示 `powersave`，本轮按 EPP 与 power profile 判定性能策略已切换。
- 性能策略切换后再次运行 `cyclictest -p 80 -m -i 1000 -l 10000 -q`：`Min 1 us / Avg 8 us / Max 60 us`；仍提示 `/dev/cpu_dma_latency` 权限不足。
- 仍需单独处理：机器人 calibration mismatch 仍存在；controller 激活阶段仍可能出现一次性 overrun warning，需继续观察是否频繁刷屏。

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
