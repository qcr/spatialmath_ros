from spatialmath import SE2, SE3, SO2, SO3, UnitQuaternion

from geometry_msgs.msg import Pose


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
