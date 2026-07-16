from dataclasses import dataclass

from geometry_msgs.msg import TransformStamped


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


def transform_point(pose: PoseTarget, point: Point3) -> Point3:
    offset = rotate_vector(pose.orientation, (point.x, point.y, point.z))
    return Point3(
        x=pose.position.x + offset[0],
        y=pose.position.y + offset[1],
        z=pose.position.z + offset[2],
    )


def pose_target_from_transform(transform: TransformStamped) -> PoseTarget:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return PoseTarget(
        position=Point3(translation.x, translation.y, translation.z),
        orientation=Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def projected_force_z_in_base(
    *,
    force_xyz: tuple[float, float, float],
    source_orientation_in_base: Quaternion,
) -> float:
    return rotate_vector(source_orientation_in_base, force_xyz)[2]
