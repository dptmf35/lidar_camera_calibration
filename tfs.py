#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

# if __name__ == '__main__':
#     rospy.init_node('tf2_listener')

#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)

#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
#         try:
#             # `/base_link`에서 `/node` 프레임까지의 변환을 얻습니다.
#             trans = tfBuffer.lookup_transform('base_link', 'zed2_camera_center', rospy.Time(0))
#             # Translation 정보 출력
#             print("Translation: x=%f, y=%f, z=%f" % (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z))
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             print(e)
#             continue

#         rate.sleep()

#!/usr/bin/env python

# < rpy >

# import rospy
# from geometry_msgs.msg import PoseStamped
# import tf.transformations

# def pose_callback(pose_msg):
#     # Quaternion to Euler conversion
#     quaternion = (
#         pose_msg.pose.orientation.x,
#         pose_msg.pose.orientation.y,
#         pose_msg.pose.orientation.z,
#         pose_msg.pose.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
    
#     roll = euler[0]
#     pitch = euler[1]
#     yaw = euler[2]
    
#     print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

# if __name__ == '__main__':
#     rospy.init_node('zed_pose_listener', anonymous=True)
#     rospy.Subscriber("/zed2/zed_node/pose", PoseStamped, pose_callback)
#     rospy.spin()


# from scipy.spatial.transform import Rotation as R
import tf
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped

# tf listener
rospy.init_node('tf_to_matrix_listener')
listener = tf.TransformListener()

listener.waitForTransform('/PandarSwift', '/zed2_left_camera_optical_frame', rospy.Time(), rospy.Duration(4.0))
(trans, rot) = listener.lookupTransform('/PandarSwift', '/zed2_left_camera_optical_frame', rospy.Time(0))
print(trans, rot)



# get 4 * 4 matrix
transformation_matrix = tf.transformations.compose_matrix(translate = trans, angles = tf.transformations.euler_from_quaternion(rot))

print("tf matrix:\n", transformation_matrix)


import numpy as np
from scipy.spatial.transform import Rotation
def create_transform_matrix(translation, rotation):
    """
    Create a transformation matrix from translation and rotation values.

    Args:
        translation (np.ndarray): Translation vector of shape (3, ).
        rotation (np.ndarray): Rotation vector of shape (3, ) or rotation matrix of shape (3, 3).

    Returns:
        transform_matrix (np.ndarray): Transformation matrix of shape (4, 4).
    """
    # Create rotation matrix from rotation vector
    if rotation.shape == (3,):
        rotation_matrix = Rotation.from_euler('xyz', rotation).as_matrix()
    else:
        rotation_matrix = rotation

    # Create transformation matrix
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    return transform_matrix

# # Example translation and rotation values
# translation = np.array([0.000, 0.075, 0.097])
# rotation = np.array([-1.571, -0.000, 3.092])  # Rotation angles in radians

# # Create transformation matrix
# transform_matrix = create_transform_matrix(translation, rotation)

# print("Transformation Matrix:")
# print(transform_matrix)


# rotation = R.from_quat(rot).as_matrix()

# # 변환 행렬 생성
# tf_matrix = np.eye(4)
# tf_matrix[:3, :3] = rotation
# tf_matrix[:3, 3] = trans

# print("Transformation Matrix:")
# print(tf_matrix)

# Translation (이동)
# translation = [-0.000, -0.112, -0.090]

# # Rotation (회전)
# rotation_quaternion = [0.477, -0.479, 0.520, 0.522]
# rotation_rpy = tf.transformations.euler_from_quaternion(rotation_quaternion)

# # 변환 행렬 생성
# tf_matrix = tf.transformations.compose_matrix(translate=translation, angles=rotation_rpy)

# print("Transformation Matrix:")
# print(tf_matrix)