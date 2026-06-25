import math
from dataclasses import dataclass

from .pen_math import PenPose2D, pen_axis_vector


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class PoseTarget:
    position: Point3
    orientation: Quaternion


def pose_target_from_pen_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
) -> PoseTarget:
    return pen_tip_pose_from_pen_pose(
        pen_pose=pen_pose,
        paper_origin=paper_origin,
        pen_length=pen_length,
    )


def pen_tip_pose_from_pen_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
) -> PoseTarget:
    axis = pen_axis_vector(
        tail_yaw=pen_pose.yaw,
        tilt_rad=pen_pose.tilt_rad,
        pen_length=pen_length,
    )
    z_axis = normalize_vector(axis)
    heading = normalize_vector((math.cos(pen_pose.yaw), math.sin(pen_pose.yaw), 0.0))
    y_axis = normalize_vector(cross(z_axis, heading))
    x_axis = normalize_vector(cross(y_axis, z_axis))

    return PoseTarget(
        position=Point3(
            x=paper_origin.x + pen_pose.tip_x,
            y=paper_origin.y + pen_pose.tip_y,
            z=paper_origin.z,
        ),
        orientation=quaternion_from_matrix_columns(x_axis, y_axis, z_axis),
    )


def tool_pose_from_pen_tip_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
    tool0_to_pen_tip_xyz: Point3,
) -> PoseTarget:
    pen_tip_target = pen_tip_pose_from_pen_pose(
        pen_pose=pen_pose,
        paper_origin=paper_origin,
        pen_length=pen_length,
    )
    offset = rotate_vector(
        pen_tip_target.orientation,
        (
            tool0_to_pen_tip_xyz.x,
            tool0_to_pen_tip_xyz.y,
            tool0_to_pen_tip_xyz.z,
        ),
    )
    return PoseTarget(
        position=Point3(
            x=pen_tip_target.position.x - offset[0],
            y=pen_tip_target.position.y - offset[1],
            z=pen_tip_target.position.z - offset[2],
        ),
        orientation=pen_tip_target.orientation,
    )


def transform_point(
    pose: PoseTarget,
    point: Point3,
) -> Point3:
    offset = rotate_vector(pose.orientation, (point.x, point.y, point.z))
    return Point3(
        x=pose.position.x + offset[0],
        y=pose.position.y + offset[1],
        z=pose.position.z + offset[2],
    )


def quaternion_from_matrix_columns(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> Quaternion:
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return normalize_quaternion(
            Quaternion(
                x=(m21 - m12) / s,
                y=(m02 - m20) / s,
                z=(m10 - m01) / s,
                w=0.25 * s,
            )
        )
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return normalize_quaternion(
            Quaternion(
                x=0.25 * s,
                y=(m01 + m10) / s,
                z=(m02 + m20) / s,
                w=(m21 - m12) / s,
            )
        )
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return normalize_quaternion(
            Quaternion(
                x=(m01 + m10) / s,
                y=0.25 * s,
                z=(m12 + m21) / s,
                w=(m02 - m20) / s,
            )
        )

    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return normalize_quaternion(
        Quaternion(
            x=(m02 + m20) / s,
            y=(m12 + m21) / s,
            z=0.25 * s,
            w=(m10 - m01) / s,
        )
    )


def rotate_vector(
    quaternion: Quaternion,
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm == 0.0:
        raise ValueError("quaternion norm must be non-zero")
    return Quaternion(
        x=quaternion.x / norm,
        y=quaternion.y / norm,
        z=quaternion.z / norm,
        w=quaternion.w / norm,
    )


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if norm == 0.0:
        raise ValueError("vector norm must be non-zero")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def cross(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
