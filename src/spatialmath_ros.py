import numpy as np
from spatialmath import SE2, SE3, UnitQuaternion

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3


def pose_msg_to_se2(pose_msg):
    return SE2(pose_msg.position.x, pose_msg.position.y) * SE2(
        UnitQuaternion(pose_msg.orientation.w, [
            pose_msg.orientation.x, pose_msg.orientation.y,
            pose_msg.orientation.z
        ]).rpy()[1])


def pose_msg_to_se3(pose_msg):
    return SE3(pose_msg.position.x, pose_msg.position.y,
               pose_msg.position.z) * UnitQuaternion(pose_msg.orientation.w, [
                   pose_msg.orientation.x, pose_msg.orientation.y,
                   pose_msg.orientation.z
               ]).SE3()


def se2_to_pose_msg(se2):
    return Pose(position=Point(*np.concatenate([se2.t, [0.0]])),
                orientation=se2_to_quat_msg(se2))


def se2_to_tf_msg(se2):
    uq = UnitQuaternion.Rz(se2.theta())
    return Transform(translation=Vector3(*np.concatenate([se2.t, [0.0]])),
                     rotation=se2_to_quat_msg(se2))


def se2_to_quat_msg(se2):
    # TODO update this once Quaternion.vec_xyzs is fixed:
    #   https://github.com/petercorke/spatialmath-python/pull/30
    uq = UnitQuaternion.Rz(se2.theta())
    return Quaternion(*np.concatenate([uq.vec3, [uq.s]]))


def se2_to_se3(se2):
    return SE3(*se2.t, 0.0) * SE3.Rz(se2.theta())


def se3_to_quat_msg(se3):
    # TODO update this once Quaternion.vec_xyzs is fixed
    #   https://github.com/petercorke/spatialmath-python/pull/30
    uq = UnitQuaternion(se3)
    return Quaternion(*np.concatenate([uq.vec3, [uq.s]]))


def se3_to_pose_msg(se3):
    return Pose(position=Point(*se3.t), orientation=se3_to_quat_msg(se3))


def se3_to_se2(se3):
    return SE2(*se3.t[0:2], se3.rpy()[2])


def se3_to_tf_msg(se3):
    return Transform(translation=Vector3(*se3.t),
                     rotation=se3_to_quat_msg(se3))


def tf_msg_to_se2(tf_msg):
    return SE2(tf_msg.translation.x, tf_msg.translation.y) * SE2(
        UnitQuaternion(tf_msg.rotation.w, [
            tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z
        ]).rpy()[1])


def tf_msg_to_se3(tf_msg):
    return SE3(tf_msg.translation.x, tf_msg.translation.y,
               tf_msg.translation.z) * UnitQuaternion(tf_msg.rotation.w, [
                   tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z
               ]).SE3()
