#!/usr/bin/env python3

import argparse
import subprocess
import sys

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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Retry the full Task 7E Servo launch until it completes successfully."
    )
    parser.add_argument("--workspace-root", default=".", help="Workspace root containing install/ and logs/task7E")
    parser.add_argument("--max-attempts", type=int, default=5)
    parser.add_argument("--timeout-sec", type=int, default=45)
    parser.add_argument("--use-mock-hardware", default="true")
    parser.add_argument("--launch-rviz", default="false")
    args = parser.parse_args()

    workspace_root = Path(args.workspace_root).resolve()
    log_root = workspace_root / "logs" / "task7E"
    log_root.mkdir(parents=True, exist_ok=True)

    launch_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {workspace_root / 'install' / 'setup.bash'} && "
        f"cd {workspace_root} && "
        "ros2 launch ur3_moveit_servo_lab_cpp task7E_moveit_servo.launch.py "
        f"use_mock_hardware:={args.use_mock_hardware} "
        f"launch_rviz:={args.launch_rviz}"
    )

    for attempt in range(1, args.max_attempts + 1):
        print(f"[task7E-test] attempt {attempt}/{args.max_attempts}")
        before = latest_log_dir(log_root)

        try:
            run = subprocess.run(
                ["bash", "-lc", launch_cmd],
                capture_output=True,
                text=True,
                timeout=args.timeout_sec,
            )
            returncode = run.returncode
        except subprocess.TimeoutExpired:
            returncode = 124

        after = latest_log_dir(log_root)
        if after is None or after == before:
            print(
                f"[task7E-test] attempt {attempt} did not create a new log directory; returncode={returncode}"
            )
            continue

        success = returncode == 0 and attempt_success(after)
        print(
            f"[task7E-test] attempt {attempt} returncode={returncode} status={'PASS' if success else 'FAIL'} log_dir={after}"
        )
        for line in collect_detail_lines(after):
            print(f"[task7E-test] {line}")

        if success:
            print(f"[task7E-test] succeeded on attempt {attempt}")
            return 0

    print("[task7E-test] all attempts failed")
    return 1


if __name__ == "__main__":
    sys.exit(main())
