#!/usr/bin/env python3

import argparse
import signal
import subprocess
import sys
import time

from pathlib import Path


SUCCESS_MARKERS = (
    "Received ServoStatus on /servo_node/status",
    "Servo command sequence finished",
)
DETAIL_MARKERS = (
    "Received ServoStatus on /servo_node/status",
    "Servo command sequence finished",
    "Timed out after 15.0s waiting for /servo_node/status",
    "Waiting to receive robot state update.",
)
START_MOTION_SERVICE = "/ur3_servo_twist_commander/start_motion"


def latest_log_dir(log_root: Path) -> Path | None:
    candidates = sorted(path for path in log_root.iterdir() if path.is_dir())
    return candidates[-1] if candidates else None


def collect_detail_lines(log_dir: Path) -> list[str]:
    details: list[str] = []
    for path in sorted(log_dir.glob("*.log")):
        try:
            text = path.read_text()
        except OSError:
            continue
        for marker in DETAIL_MARKERS:
            if marker in text:
                for line in text.splitlines():
                    if marker in line:
                        details.append(f"{path.name}: {line}")
        if len(details) >= 20:
            break
    return details[-20:]


def attempt_success(log_dir: Path) -> bool:
    markers_found = {marker: False for marker in SUCCESS_MARKERS}
    for path in log_dir.glob("*.log"):
        try:
            text = path.read_text()
        except OSError:
            continue
        for marker in SUCCESS_MARKERS:
            if marker in text:
                markers_found[marker] = True
    return all(markers_found.values())


def marker_count(log_dir: Path, marker: str) -> int:
    count = 0
    for path in log_dir.glob("*.log"):
        try:
            text = path.read_text()
        except OSError:
            continue
        count += text.count(marker)
    return count


def build_env_command(workspace_root: Path, command: str) -> str:
    return (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {workspace_root / 'install' / 'setup.bash'} && "
        f"cd {workspace_root} && "
        f"{command}"
    )


def wait_for_new_log_dir(log_root: Path, before: Path | None, timeout_sec: float) -> Path | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        after = latest_log_dir(log_root)
        if after is not None and after != before:
            return after
        time.sleep(0.2)
    return None


def wait_for_service(workspace_root: Path, service_name: str, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    cmd = build_env_command(workspace_root, f"ros2 service type {service_name}")
    while time.monotonic() < deadline:
        run = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
        )
        if run.returncode == 0 and "std_srvs/srv/Trigger" in run.stdout:
            return True
        time.sleep(0.5)
    return False


def call_start_motion(workspace_root: Path) -> tuple[bool, str]:
    cmd = build_env_command(
        workspace_root,
        f"ros2 service call {START_MOTION_SERVICE} std_srvs/srv/Trigger '{{}}'",
    )
    run = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=15,
    )
    output = f"{run.stdout}\n{run.stderr}"
    success = (
        run.returncode == 0
        and (
            "success=True" in output
            or "success: True" in output
            or "success: true" in output
        )
    )
    return success, output.strip()


def wait_for_completion_count(log_dir: Path, expected_count: int, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if marker_count(log_dir, "Servo command sequence finished") >= expected_count:
            return True
        time.sleep(0.5)
    return False


def stop_launch(process: subprocess.Popen[str], timeout_sec: float) -> int:
    if process.poll() is not None:
        return process.returncode

    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait(timeout=5)


def remaining_time(deadline: float) -> float:
    return max(0.0, deadline - time.monotonic())


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Retry the full Task 7E Servo launch until it completes successfully."
    )
    parser.add_argument("--workspace-root", default=".", help="Workspace root containing install/ and logs/task7E")
    parser.add_argument("--max-attempts", type=int, default=5)
    parser.add_argument("--timeout-sec", type=int, default=45)
    parser.add_argument("--use-mock-hardware", default="true")
    parser.add_argument("--launch-rviz", default="false")
    parser.add_argument("--trigger-count", type=int, default=2)
    args = parser.parse_args()

    workspace_root = Path(args.workspace_root).resolve()
    log_root = workspace_root / "logs" / "task7E"
    log_root.mkdir(parents=True, exist_ok=True)

    launch_cmd = build_env_command(
        workspace_root,
        "ros2 launch ur3_moveit_servo_lab_cpp task7E_moveit_servo.launch.py "
        f"use_mock_hardware:={args.use_mock_hardware} "
        f"launch_rviz:={args.launch_rviz}",
    )

    for attempt in range(1, args.max_attempts + 1):
        print(f"[task7E-test] attempt {attempt}/{args.max_attempts}")
        before = latest_log_dir(log_root)
        after: Path | None = None
        success = False
        last_call_output = ""
        attempt_deadline = time.monotonic() + args.timeout_sec
        launch_process = subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )

        try:
            after = wait_for_new_log_dir(
                log_root,
                before,
                timeout_sec=min(5.0, remaining_time(attempt_deadline)),
            )
            if after is None:
                returncode = stop_launch(launch_process, timeout_sec=5.0)
                print(
                    f"[task7E-test] attempt {attempt} did not create a new log directory; returncode={returncode}"
                )
                continue

            if not wait_for_service(
                workspace_root,
                START_MOTION_SERVICE,
                timeout_sec=min(20.0, remaining_time(attempt_deadline)),
            ):
                returncode = stop_launch(launch_process, timeout_sec=5.0)
                print(
                    f"[task7E-test] attempt {attempt} service {START_MOTION_SERVICE} was not available; returncode={returncode}"
                )
                for line in collect_detail_lines(after):
                    print(f"[task7E-test] {line}")
                continue

            success = True
            for trigger_index in range(1, args.trigger_count + 1):
                trigger_ok = False
                retry_deadline = min(time.monotonic() + 20.0, attempt_deadline)
                while time.monotonic() < retry_deadline:
                    if launch_process.poll() is not None:
                        success = False
                        break
                    trigger_ok, last_call_output = call_start_motion(workspace_root)
                    if trigger_ok:
                        print(f"[task7E-test] trigger {trigger_index}/{args.trigger_count} accepted")
                        break
                    time.sleep(0.5)

                if not success or not trigger_ok:
                    success = False
                    break

                if not wait_for_completion_count(
                    after,
                    trigger_index,
                    timeout_sec=min(20.0, remaining_time(attempt_deadline)),
                ):
                    success = False
                    break

            returncode = stop_launch(launch_process, timeout_sec=5.0)
        except subprocess.TimeoutExpired:
            launch_process.kill()
            returncode = launch_process.wait(timeout=5)
            success = False

        if after is None:
            print(
                f"[task7E-test] attempt {attempt} did not create a new log directory; returncode={returncode}"
            )
            continue

        success = success and attempt_success(after)
        display_returncode = 0 if success else returncode
        print(
            f"[task7E-test] attempt {attempt} returncode={display_returncode} status={'PASS' if success else 'FAIL'} log_dir={after}"
        )
        if not success and last_call_output:
            print(f"[task7E-test] last start_motion output:\n{last_call_output}")
        for line in collect_detail_lines(after):
            print(f"[task7E-test] {line}")

        if success:
            print(f"[task7E-test] succeeded on attempt {attempt}")
            return 0

    print("[task7E-test] all attempts failed")
    return 1


if __name__ == "__main__":
    sys.exit(main())
