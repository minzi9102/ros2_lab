#!/usr/bin/env python3

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode, IsInRemoteControl
from ur_dashboard_msgs.srv import IsProgramRunning


@dataclass
class GateResult:
    category: str
    check: str
    current: str
    level: str
    note: str


class RealRobotStateCheck(Node):
    def __init__(self) -> None:
        super().__init__("task8c_state_check")
        self.declare_parameter("require_external_control", True)
        self.declare_parameter("require_trajectory_controller_active", False)
        self.declare_parameter("report_only", True)
        self.declare_parameter("sample_seconds", 3.0)
        self.declare_parameter("robot_mode_service", "/dashboard_client/get_robot_mode")
        self.declare_parameter("safety_mode_service", "/dashboard_client/get_safety_mode")
        self.declare_parameter(
            "program_running_service", "/dashboard_client/program_running"
        )
        self.declare_parameter(
            "remote_control_service", "/dashboard_client/is_in_remote_control"
        )
        self.declare_parameter(
            "list_controllers_service", "/controller_manager/list_controllers"
        )

        self._joint_sample_count = 0
        self._first_joint_sample_time: Optional[float] = None
        self._last_joint_sample_time: Optional[float] = None
        self._last_joint_state: Optional[JointState] = None
        self._last_speed_scaling: Optional[float] = None

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(
            Float64,
            "/speed_scaling_state_broadcaster/speed_scaling",
            self._on_speed_scaling,
            10,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        if self._first_joint_sample_time is None:
            self._first_joint_sample_time = now
        self._last_joint_sample_time = now
        self._joint_sample_count += 1
        self._last_joint_state = msg

    def _on_speed_scaling(self, msg: Float64) -> None:
        self._last_speed_scaling = msg.data

    def report(self) -> None:
        require_external_control = self.get_parameter(
            "require_external_control"
        ).get_parameter_value().bool_value
        require_trajectory_controller_active = self.get_parameter(
            "require_trajectory_controller_active"
        ).get_parameter_value().bool_value
        report_only = self.get_parameter("report_only").get_parameter_value().bool_value
        sample_seconds = (
            self.get_parameter("sample_seconds").get_parameter_value().double_value
        )

        self.get_logger().info("Task 8C state-check started.")
        self.get_logger().info(f"require_external_control={require_external_control}")
        self.get_logger().info(
            "require_trajectory_controller_active="
            f"{require_trajectory_controller_active}"
        )
        self.get_logger().info(f"report_only={report_only}")
        self.get_logger().info(
            "This checker only reads services/topics and never sends motion commands."
        )

        robot_mode = self._call_service(
            GetRobotMode,
            self._param_string("robot_mode_service"),
            GetRobotMode.Request(),
        )
        safety_mode = self._call_service(
            GetSafetyMode,
            self._param_string("safety_mode_service"),
            GetSafetyMode.Request(),
        )
        program_running = self._call_service(
            IsProgramRunning,
            self._param_string("program_running_service"),
            IsProgramRunning.Request(),
        )
        remote_control = self._call_service(
            IsInRemoteControl,
            self._param_string("remote_control_service"),
            IsInRemoteControl.Request(),
        )
        controllers = self._call_service(
            ListControllers,
            self._param_string("list_controllers_service"),
            ListControllers.Request(),
        )

        self.get_logger().info(f"Sampling /joint_states for {sample_seconds:.1f}s...")
        end_time = time.monotonic() + sample_seconds
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        results = [
            self._classify_robot_mode(robot_mode),
            self._classify_safety_mode(safety_mode),
            self._classify_program(program_running, require_external_control),
            self._classify_remote_control(remote_control),
            self._classify_joint_state_broadcaster(controllers),
            self._classify_trajectory_controller(
                controllers, require_trajectory_controller_active
            ),
            self._classify_joint_states(),
            self._classify_speed_scaling(),
        ]

        self._print_matrix(results)
        if any(result.level == "block" for result in results):
            self.get_logger().error("Task 8C gate result: BLOCK")
        elif any(result.level == "warn" for result in results):
            self.get_logger().warn("Task 8C gate result: WARN")
        else:
            self.get_logger().info("Task 8C gate result: PASS")

    def _param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _call_service(self, service_type: Any, name: str, request: Any) -> Any:
        client = self.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service unavailable: {name}")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            return future.result()
        self.get_logger().error(f"Service timeout: {name}")
        return None

    def _classify_robot_mode(self, response: Any) -> GateResult:
        if response is None or not response.success:
            return GateResult("Dashboard", "robot mode", "unavailable", "block", "服务查询失败")
        mode = response.robot_mode.mode
        if mode == RobotMode.RUNNING:
            level = "pass"
            note = "机器人处于 RUNNING"
        elif mode in (RobotMode.POWER_ON, RobotMode.IDLE):
            level = "warn"
            note = "机器人已上电但未处于 RUNNING，需要人工解释"
        else:
            level = "block"
            note = "机器人不在可控运行状态"
        return GateResult("Dashboard", "robot mode", response.answer, level, note)

    def _classify_safety_mode(self, response: Any) -> GateResult:
        if response is None or not response.success:
            return GateResult("Dashboard", "safety mode", "unavailable", "block", "服务查询失败")
        mode = response.safety_mode.mode
        if mode == SafetyMode.NORMAL:
            level = "pass"
            note = "当前 safety mode 为 NORMAL"
        elif mode == SafetyMode.REDUCED:
            level = "warn"
            note = "处于 REDUCED，需要记录触发条件与现场含义"
        else:
            level = "block"
            note = "安全模式需要人工恢复或排障"
        return GateResult("Dashboard", "safety mode", response.answer, level, note)

    def _classify_program(self, response: Any, required: bool) -> GateResult:
        if response is None or not response.success:
            return GateResult("Program", "External Control", "unavailable", "block", "服务查询失败")
        running = response.program_running
        if running:
            level = "pass"
            note = "External Control 程序正在运行"
        elif required:
            level = "block"
            note = "要求 External Control 运行，但当前未运行"
        else:
            level = "warn"
            note = "当前未运行 External Control"
        return GateResult("Program", "External Control", response.answer, level, note)

    def _classify_remote_control(self, response: Any) -> GateResult:
        if response is None or not response.success:
            return GateResult("Dashboard", "remote control", "unavailable", "warn", "服务查询失败")
        if response.remote_control:
            level = "pass"
            note = "Dashboard 远程控制可用"
        else:
            level = "warn"
            note = "当前不是 Remote Control；示教器启动可接受，远程 load/play 不可假设"
        return GateResult(
            "Dashboard",
            "remote control",
            f"remote_control={response.remote_control}",
            level,
            note,
        )

    def _classify_joint_state_broadcaster(self, response: Any) -> GateResult:
        state = self._controller_state(response, "joint_state_broadcaster")
        if state == "active":
            return GateResult("Controller", "joint state broadcaster", state, "pass", "状态流 broadcaster active")
        if state is None:
            return GateResult("Controller", "joint state broadcaster", "missing", "block", "controller 缺失")
        return GateResult("Controller", "joint state broadcaster", state, "block", "必须 active 才能观察状态")

    def _classify_trajectory_controller(self, response: Any, required_active: bool) -> GateResult:
        state = self._controller_state(response, "scaled_joint_trajectory_controller")
        if state == "active":
            return GateResult("Controller", "trajectory controller", state, "pass", "轨迹 controller active")
        if state is None:
            return GateResult("Controller", "trajectory controller", "missing", "block", "controller 缺失")
        if required_active:
            return GateResult("Controller", "trajectory controller", state, "block", "进入动作任务前要求 active")
        return GateResult(
            "Controller",
            "trajectory controller",
            state,
            "warn",
            "8C 只读阶段故意保持 inactive；进入 8D 前需重新决策",
        )

    def _controller_state(self, response: Any, name: str) -> Optional[str]:
        if response is None:
            return None
        for controller in response.controller:
            if controller.name == name:
                return controller.state
        return None

    def _classify_joint_states(self) -> GateResult:
        if self._joint_sample_count == 0 or self._last_joint_state is None:
            return GateResult("State", "/joint_states", "no samples", "block", "没有收到 JointState")
        rate = self._joint_state_rate()
        names = ", ".join(self._last_joint_state.name)
        if rate is not None and rate >= 100.0:
            level = "pass"
            note = f"收到 {self._joint_sample_count} 条样本，关节名: {names}"
        else:
            level = "warn"
            note = f"收到 {self._joint_sample_count} 条样本，频率偏低或样本不足"
        current = f"{rate:.1f} Hz" if rate is not None else "sampled"
        return GateResult("State", "/joint_states", current, level, note)

    def _joint_state_rate(self) -> Optional[float]:
        if (
            self._first_joint_sample_time is None
            or self._last_joint_sample_time is None
            or self._joint_sample_count < 2
        ):
            return None
        duration = self._last_joint_sample_time - self._first_joint_sample_time
        if duration <= 0.0:
            return None
        return (self._joint_sample_count - 1) / duration

    def _classify_speed_scaling(self) -> GateResult:
        if self._last_speed_scaling is None:
            return GateResult("State", "speed scaling", "no sample", "warn", "未收到 speed scaling")
        if self._last_speed_scaling > 0.0:
            level = "pass"
            note = "speed scaling 非零，External Control 链路处于可解释状态"
        else:
            level = "block"
            note = "speed scaling 为 0，进入动作任务前必须解释"
        return GateResult(
            "State",
            "speed scaling",
            f"{self._last_speed_scaling:.1f}",
            level,
            note,
        )

    def _print_matrix(self, results: list[GateResult]) -> None:
        self.get_logger().info("Task 8C pass/warn/block matrix:")
        for result in results:
            line = (
                f"[{result.level.upper()}] {result.category} / {result.check}: "
                f"{result.current} -- {result.note}"
            )
            if result.level == "block":
                self.get_logger().error(line)
            elif result.level == "warn":
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        self.get_logger().warn(
            "This checker never unlocks protective stop, restarts safety, or sends motion commands."
        )


def main() -> int:
    rclpy.init()
    node = RealRobotStateCheck()
    try:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
