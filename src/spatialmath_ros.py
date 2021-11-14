import numpy as np
from spatialmath import SE2, SE3, Twist2, Twist3, UnitQuaternion

from geometry_msgs.msg import (Point, Pose, Quaternion, Transform, Twist,
                               Vector3)


def pose_msg_to_se2(pose_msg):
    return SE2(pose_msg.position.x, pose_msg.position.y) * SE2(
        UnitQuaternion(pose_msg.orientation.w, [
            pose_msg.orientation.x, pose_msg.orientation.y,
            pose_msg.orientation.z
        ]).rpy()[2])


def pose_msg_to_se3(pose_msg):
    return SE3(pose_msg.position.x, pose_msg.position.y,
               pose_msg.position.z) * UnitQuaternion(pose_msg.orientation.w, [
                   pose_msg.orientation.x, pose_msg.orientation.y,
                   pose_msg.orientation.z
               ]).SE3()


def quat_to_quat_msg(quat):
    # TODO update this once Quaternion.vec_xyzs is fixed
    #   https://github.com/petercorke/spatialmath-python/pull/30
    return Quaternion(*np.concatenate([quat.vec3, [quat.s]]))


def quat_msg_to_quat(quat_msg):
    return UnitQuaternion(quat_msg.w, [quat_msg.x, quat_msg.y, quat_msg.z])


def se2_to_pose_msg(se2):
    return Pose(position=Point(*np.concatenate([se2.t, [0.0]])),
                orientation=se2_to_quat_msg(se2))


def se2_to_quat_msg(se2):
    return quat_to_quat_msg(UnitQuaternion.Rz(se2.theta()))


def se2_to_se3(se2):
    return SE3(*se2.t, 0.0) * SE3.Rz(se2.theta())


def se2_to_tf_msg(se2):
    uq = UnitQuaternion.Rz(se2.theta())
    return Transform(translation=Vector3(*np.concatenate([se2.t, [0.0]])),
                     rotation=se2_to_quat_msg(se2))


def se3_to_pose_msg(se3):
    return Pose(position=Point(*se3.t), orientation=se3_to_quat_msg(se3))


def se3_to_quat_msg(se3):
    uq = UnitQuaternion(se3)
    return Quaternion(*np.concatenate([uq.vec3, [uq.s]]))


def se3_to_se2(se3):
    return SE2(*se3.t[0:2], se3.rpy()[2])


def se3_to_tf_msg(se3):
    return Transform(translation=Vector3(*se3.t),
                     rotation=se3_to_quat_msg(se3))


def tf_msg_to_se2(tf_msg):
    return SE2(tf_msg.translation.x, tf_msg.translation.y) * SE2(
        UnitQuaternion(tf_msg.rotation.w, [
            tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z
        ]).rpy()[2])


def tf_msg_to_se3(tf_msg):
    return SE3(tf_msg.translation.x, tf_msg.translation.y,
               tf_msg.translation.z) * UnitQuaternion(tf_msg.rotation.w, [
                   tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z
               ]).SE3()


def twist_msg_to_twist2(twist_msg):
    return Twist2(
        [twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z])


def twist_msg_to_twist3(twist_msg):
    return Twist3([
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z
    ])


def twist2_to_twist_msg(twist2):
    return Twist(linear=Vector3(*twist2.v, 0.0), angular=Vector3(z=twist2.w))


def twist3_to_twist_msg(twist3):
    return Twist(linear=Vector3(*twist3.v), angular=Vector3(*twist3.w))
