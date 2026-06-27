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


@dataclass(frozen=True)
class OrientationFrame:
    x_axis: tuple[float, float, float]
    y_axis: tuple[float, float, float]
    z_axis: tuple[float, float, float]


class ContinuousPenOrientation:
    def __init__(
        self,
        *,
        initial_pen_pose: PenPose2D,
        pen_length: float,
        max_axis_angular_speed_radps: float,
    ) -> None:
        if max_axis_angular_speed_radps <= 0.0:
            raise ValueError(
                "max_axis_angular_speed_radps must be greater than zero"
            )
        self._pen_length = float(pen_length)
        self._max_axis_angular_speed_radps = float(
            max_axis_angular_speed_radps
        )
        self._frame = orientation_frame_from_pen_pose(
            pen_pose=initial_pen_pose,
            pen_length=self._pen_length,
        )
        self._orientation = quaternion_from_orientation_frame(self._frame)
        self._axis_error_rad = 0.0

    @property
    def orientation(self) -> Quaternion:
        return self._orientation

    @property
    def frame(self) -> OrientationFrame:
        return self._frame

    @property
    def axis_error_rad(self) -> float:
        return self._axis_error_rad

    def update(self, pen_pose: PenPose2D, dt_sec: float) -> Quaternion:
        if dt_sec < 0.0:
            raise ValueError("dt_sec must be non-negative")

        desired_frame = orientation_frame_from_pen_pose(
            pen_pose=pen_pose,
            pen_length=self._pen_length,
        )
        next_z = rotate_unit_vector_toward(
            current=self._frame.z_axis,
            target=desired_frame.z_axis,
            max_angle_rad=self._max_axis_angular_speed_radps * dt_sec,
        )
        self._frame = minimal_twist_frame(
            previous=self._frame,
            next_z_axis=next_z,
        )
        next_orientation = quaternion_from_orientation_frame(self._frame)
        if quaternion_dot(self._orientation, next_orientation) < 0.0:
            next_orientation = negate_quaternion(next_orientation)
        self._orientation = next_orientation
        self._axis_error_rad = vector_angle(next_z, desired_frame.z_axis)
        return self._orientation


def pose_target_from_pen_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
    orientation_override: Quaternion | None = None,
) -> PoseTarget:
    return pen_tip_pose_from_pen_pose(
        pen_pose=pen_pose,
        paper_origin=paper_origin,
        pen_length=pen_length,
        orientation_override=orientation_override,
    )


def pen_tip_pose_from_pen_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
    orientation_override: Quaternion | None = None,
) -> PoseTarget:
    orientation = orientation_override
    if orientation is None:
        orientation = quaternion_from_orientation_frame(
            orientation_frame_from_pen_pose(
                pen_pose=pen_pose,
                pen_length=pen_length,
            )
        )

    return PoseTarget(
        position=Point3(
            x=paper_origin.x + pen_pose.tip_x,
            y=paper_origin.y + pen_pose.tip_y,
            z=paper_origin.z,
        ),
        orientation=orientation,
    )


def tool_pose_from_pen_tip_pose(
    *,
    pen_pose: PenPose2D,
    paper_origin: Point3,
    pen_length: float,
    tool0_to_pen_tip_xyz: Point3,
    orientation_override: Quaternion | None = None,
) -> PoseTarget:
    pen_tip_target = pen_tip_pose_from_pen_pose(
        pen_pose=pen_pose,
        paper_origin=paper_origin,
        pen_length=pen_length,
        orientation_override=orientation_override,
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


def orientation_frame_from_pen_pose(
    *,
    pen_pose: PenPose2D,
    pen_length: float,
) -> OrientationFrame:
    tip_to_tail_axis = pen_axis_vector(
        tail_yaw=pen_pose.yaw,
        tilt_rad=pen_pose.tilt_rad,
        pen_length=pen_length,
    )
    z_axis = normalize_vector(
        (-tip_to_tail_axis[0], -tip_to_tail_axis[1], -tip_to_tail_axis[2])
    )
    heading = normalize_vector(
        (-math.cos(pen_pose.yaw), -math.sin(pen_pose.yaw), 0.0)
    )
    y_axis = normalize_vector(cross(z_axis, heading))
    x_axis = normalize_vector(cross(y_axis, z_axis))
    return OrientationFrame(x_axis=x_axis, y_axis=y_axis, z_axis=z_axis)


def quaternion_from_orientation_frame(frame: OrientationFrame) -> Quaternion:
    return quaternion_from_matrix_columns(
        frame.x_axis,
        frame.y_axis,
        frame.z_axis,
    )


def minimal_twist_frame(
    *,
    previous: OrientationFrame,
    next_z_axis: tuple[float, float, float],
) -> OrientationFrame:
    z_axis = normalize_vector(next_z_axis)
    projected_x = project_vector_onto_plane(previous.x_axis, z_axis)
    if vector_norm(projected_x) < 1e-9:
        projected_x = project_vector_onto_plane(previous.y_axis, z_axis)
    if vector_norm(projected_x) < 1e-9:
        projected_x = project_vector_onto_plane(
            least_aligned_world_axis(z_axis),
            z_axis,
        )

    x_axis = normalize_vector(projected_x)
    y_axis = normalize_vector(cross(z_axis, x_axis))
    x_axis = normalize_vector(cross(y_axis, z_axis))
    return OrientationFrame(x_axis=x_axis, y_axis=y_axis, z_axis=z_axis)


def rotate_unit_vector_toward(
    *,
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    max_angle_rad: float,
) -> tuple[float, float, float]:
    if max_angle_rad < 0.0:
        raise ValueError("max_angle_rad must be non-negative")
    current_unit = normalize_vector(current)
    target_unit = normalize_vector(target)
    angle = vector_angle(current_unit, target_unit)
    if angle <= max_angle_rad or angle < 1e-12:
        return target_unit
    if max_angle_rad == 0.0:
        return current_unit

    rotation_axis = cross(current_unit, target_unit)
    if vector_norm(rotation_axis) < 1e-9:
        rotation_axis = cross(
            current_unit,
            least_aligned_world_axis(current_unit),
        )
    rotation_axis = normalize_vector(rotation_axis)
    return normalize_vector(
        rodrigues_rotate(
            vector=current_unit,
            axis=rotation_axis,
            angle_rad=max_angle_rad,
        )
    )


def rodrigues_rotate(
    *,
    vector: tuple[float, float, float],
    axis: tuple[float, float, float],
    angle_rad: float,
) -> tuple[float, float, float]:
    axis_unit = normalize_vector(axis)
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    axis_cross_vector = cross(axis_unit, vector)
    axis_dot_vector = dot(axis_unit, vector)
    return (
        vector[0] * cosine
        + axis_cross_vector[0] * sine
        + axis_unit[0] * axis_dot_vector * (1.0 - cosine),
        vector[1] * cosine
        + axis_cross_vector[1] * sine
        + axis_unit[1] * axis_dot_vector * (1.0 - cosine),
        vector[2] * cosine
        + axis_cross_vector[2] * sine
        + axis_unit[2] * axis_dot_vector * (1.0 - cosine),
    )


def project_vector_onto_plane(
    vector: tuple[float, float, float],
    plane_normal: tuple[float, float, float],
) -> tuple[float, float, float]:
    normal = normalize_vector(plane_normal)
    normal_component = dot(vector, normal)
    return (
        vector[0] - normal_component * normal[0],
        vector[1] - normal_component * normal[1],
        vector[2] - normal_component * normal[2],
    )


def least_aligned_world_axis(
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    return min(axes, key=lambda axis: abs(dot(vector, axis)))


def vector_angle(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    cosine = dot(normalize_vector(a), normalize_vector(b))
    return math.acos(clamp(cosine, -1.0, 1.0))


def vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(dot(vector, vector))


def dot(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def quaternion_dot(a: Quaternion, b: Quaternion) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w


def negate_quaternion(quaternion: Quaternion) -> Quaternion:
    return Quaternion(
        x=-quaternion.x,
        y=-quaternion.y,
        z=-quaternion.z,
        w=-quaternion.w,
    )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


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
