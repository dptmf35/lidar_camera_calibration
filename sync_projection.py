import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from datetime import datetime
import os, cv2, numpy as np
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

def voxel_downsampling(lidar_data, voxel_size=0.05):
    voxel_indices = np.floor(lidar_data[:, :3] / voxel_size).astype(int)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    filtered_data = lidar_data[unique_indices]
    
    return filtered_data

# camera intrinsic parameter
camera_matrix = np.array([[263.16546630859375, 0.0, 321.3332824707031],
                            [0.0, 263.16546630859375, 188.22222900390625],
                            [0.0, 0.0, 1.0]])



extrinsic_matrix = np.array( [[-1.00000000e+00,  1.83583740e-07, -3.66861455e-06,  6.00003687e-02],
 [ 3.67320510e-06,  4.99791693e-02, -9.98750260e-01,  1.00387282e-01],
 [ 5.69184965e-18, -9.98750260e-01, -4.99791693e-02,  1.37299792e-01],
 [0, 0, 0, 1]])


# rpy = np.array([-1.556898839059469, -0.048599870562104186, -3.109222283107868])
# rotation_matrix = np.array([[-0.998296,   -0.04810083,  0.03303637],
# [-0.0323265,  -0.01546191, -0.99935776],
# [ 0.04858074, -0.9987228,   0.01388063]]
# )
# translation = np.array([  6.00003687e-02, 1.00387282e-01, 1.37299792e-01])

# tf_matrix = np.eye(4)
# tf_matrix[:3, :3] = rotation_matrix
# tf_matrix[:3, 3] = translation

# extrinsic_matrix = tf_matrix.copy()


bridge = CvBridge()


def callback(image, pcl_data):
    # global camera_data, lidar_data
    # synchronized messages
    camera_data = bridge.imgmsg_to_cv2(image, "bgr8")
    lidar_data = np.array(list(pc2.read_points(pcl_data, skip_nans=True, field_names=('x', 'y', 'z'))))
    
    filtered_data = voxel_downsampling(lidar_data)
    lidar_points = np.hstack((filtered_data[:, :3], np.ones((filtered_data.shape[0], 1))))
    lidar_points = np.transpose(lidar_points) 
    lidar_points_cam = np.dot(extrinsic_matrix, lidar_points)  # lidar to camera coordinate

    # projected point to image coordinate 
    projected_points = np.dot(camera_matrix, lidar_points_cam[:3, :])
    projected_points = np.transpose(projected_points)  
    projected_points = projected_points[:, :2] / projected_points[:, 2:]

    for i, point in enumerate(projected_points):
        x, y = int(point[0]), int(point[1])
        if 0 <= x < camera_data.shape[1] and 0 <= y < camera_data.shape[0]:
            camera_data = cv2.circle(camera_data, (x, y), 2, (0, 255, 0), -1)


    cv2.imshow('Camera with LiDAR', camera_data)
    cv2.waitKey(1)



def listener():
    rospy.init_node('syn_projection', anonymous=True)

    image_sub = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
    pcl_sub = message_filters.Subscriber('/hesai/pandar_points', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
