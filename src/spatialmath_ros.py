import numpy as np
from spatialmath import SE2, SE3, SO2, SO3, UnitQuaternion

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3


def pose_msg_to_se2(pose_msg):
    return SE2(pose_msg.position.x, pose_msg.position.y) * SO2(
        UnitQuaternion(pose_msg.orientation.w, [
            pose_msg.orientation.x, pose_msg.orientation.y,
            pose_msg.orientation.z
        ]).rpy()[1])


def pose_msg_to_se3(pose_msg):
    return SE3(pose_msg.position.x, pose_msg.position.y,
               pose_msg.position.z) * SO3(
                   UnitQuaternion(pose_msg.orientation.w, [
                       pose_msg.orientation.x, pose_msg.orientation.y,
                       pose_msg.orientation.z
                   ]))


def se2_to_pose_msg(se2):
    return Pose(position=Point(*np.concatenate([se2.t, [0.0]])),
                orientation=se2_to_quat_msg(se2))


def se2_to_tf_msg(se2):
    uq = UnitQuaternion.Rz(se2.theta())
    return Transform(translation=Vector3(*np.concatenate([se2.t, [0.0]])),
                     rotation=se2_to_quat_msg(se2))


def se2_to_quat_msg(se2):
    # TODO update this once Quaternion.vec_xyzs is fixed
    uq = UnitQuaternion.Rz(se2.theta())
    return Quaternion(*np.concatenate([uq.vec3, [uq.s]]))


def se3_to_quat_msg(se3):
    # TODO update this once Quaternion.vec_xyzs is fixed
    uq = UnitQuaternion(se3)
    return Quaternion(*np.concatenate([uq.vec3, [uq.s]]))


def se3_to_pose_msg(se3):
    return Pose(position=Point(*se3.t),
                orientation=Quaternion(se3_to_quat_msg))


def se3_to_tf_msg(se3):
    return Transform(translation=Vector3(*se3.t),
                     rotation=se3_to_quat_msg(se3))


def tf_msg_to_se2(tf_msg):
    return SE2(tf_msg.translation.x, tf_msg.translation.y) * SO2(
        UnitQuaternion(tf_msg.rotation.w, [
            tf_msg.rotation.x, tf_msg.orientation.y, tf_msg.orientation.z
        ]).rpy()[1])


def tf_msg_to_se3(tf_msg):
    return SE3(
        tf_msg.translation.x, tf_msg.translation.y,
        tf_msg.translation.z) * SO3(
            UnitQuaternion(tf_msg.rotation.w, [
                tf_msg.rotation.x, tf_msg.orientation.y, tf_msg.orientation.z
            ]))
