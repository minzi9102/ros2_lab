import math
from dataclasses import dataclass

from .safety_limiter import TwistCommand


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


class AutoYawController:
    def __init__(
        self,
        *,
        yaw_gain: float,
        max_angular_speed_radps: float,
        min_linear_speed_mps: float,
    ) -> None:
        if yaw_gain <= 0.0:
            raise ValueError('auto_yaw_gain must be greater than zero')
        if max_angular_speed_radps <= 0.0:
            raise ValueError('max_angular_speed_radps must be greater than zero')
        if min_linear_speed_mps < 0.0:
            raise ValueError('auto_yaw_min_linear_speed_mps must be non-negative')

        self.yaw_gain = float(yaw_gain)
        self.max_angular_speed_radps = float(max_angular_speed_radps)
        self.min_linear_speed_mps = float(min_linear_speed_mps)

    def apply(self, command: TwistCommand, current_tool_yaw: float | None) -> TwistCommand:
        if current_tool_yaw is None:
            return command

        speed = math.hypot(command.linear_x, command.linear_y)
        if speed < self.min_linear_speed_mps:
            return command

        desired_yaw = desired_reverse_motion_yaw(command.linear_x, command.linear_y)
        error = normalize_angle(desired_yaw - current_tool_yaw)
        angular_z = clamp(
            self.yaw_gain * error,
            -self.max_angular_speed_radps,
            self.max_angular_speed_radps,
        )
        return TwistCommand(
            linear_x=command.linear_x,
            linear_y=command.linear_y,
            linear_z=command.linear_z,
            angular_x=command.angular_x,
            angular_y=command.angular_y,
            angular_z=angular_z,
        )


def desired_reverse_motion_yaw(linear_x: float, linear_y: float) -> float:
    yaw = math.atan2(-linear_y, -linear_x)
    if math.isclose(yaw, -math.pi):
        return math.pi
    return yaw


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
