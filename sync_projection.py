import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from datetime import datetime
import os, cv2, numpy as np
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
# from scipy.spatial import cKDTree
import time 


def voxel_downsampling(lidar_data, voxel_size=0.1):
    # start = time.time()
    points = np.array(lidar_data)
    voxel_indices = np.floor(lidar_data[:, :3] / voxel_size).astype(int)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    filtered_data = lidar_data[unique_indices]
    # print(f"voxel grid processing time : {time.time() - start:.2f}")
    return filtered_data


# camera intrinsic parameter
camera_matrix = np.array([[263.16546630859375, 0.0, 321.3332824707031],
                            [0.0, 263.16546630859375, 188.22222900390625],
                            [0.0, 0.0, 1.0]])

extrinsic_matrix = np.array(
[[-0.999649,-0.0248669,0.00912824,0.05193],
[-0.0102284,0.044481,-0.998957,0.131159],
[0.024435,-0.998701,-0.0447198,0.0918986],
[0,0,0,1]])

bridge = CvBridge()
output_pub = rospy.Publisher('/output', Image, queue_size=10)


def callback(image, pcl_data):
    # Convert ROS Image message to OpenCV image
    start = time.time()
    camera_data = bridge.imgmsg_to_cv2(image, "bgr8")
    # Read x, y, z, and intensity fields from LiDAR point cloud data
    lidar_data = np.array(list(pc2.read_points(pcl_data, skip_nans=True, field_names=('x', 'y', 'z', 'intensity'))))
    lidar_data = lidar_data[(lidar_data[:, 0] != 0) | (lidar_data[:, 1] != 0) | (lidar_data[:, 2] != 0)] # nonzero points
    filtered_data = voxel_downsampling(lidar_data[:, :3], 0.225)  # Only use x, y, z for downsampling

    # print(f"{lidar_data.shape} ---> {filtered_data.shape}")
    # filtered_data = lidar_data[:, :3]
    lidar_points = np.hstack((filtered_data, np.ones((filtered_data.shape[0], 1))))
    lidar_points = np.transpose(lidar_points)
    lidar_points_cam = np.dot(extrinsic_matrix, lidar_points)  # LiDAR points -> camera coordinate system

    # LiDAR points -> image coordinates
    projected_points = np.dot(camera_matrix, lidar_points_cam[:3, :])
    projected_points = np.transpose(projected_points)
    projected_points = projected_points[:, :2] / projected_points[:, 2:]

    # print(f'dot product processing time : {time.time() - start:.2f}')


    # Find the minimum and maximum intensity values for normalization
    min_intensity = np.min(lidar_data[:, 3])
    max_intensity = np.max(lidar_data[:, 3])

    for i, point in enumerate(projected_points):
        x, y = int(point[0]), int(point[1])
        intensity = lidar_data[i, 3]
        color = intensity_to_color(intensity, min_intensity, max_intensity)
        if 0 <= x < camera_data.shape[1] and 0 <= y < camera_data.shape[0]:
            camera_data = cv2.circle(camera_data, (x, y), 2, color, -1)
    processing_time = time.time() - start

    fps = 1.0 / processing_time
    rospy.loginfo(f"FPS : {fps:.2f}")
    # cv2.imshow('Camera with LiDAR', camera_data)
    # k = cv2.waitKey(1) & 0xFF

    # if k == 27 or k == ord('q'):  # ESC or q key 
    #     cv2.destroyAllWindows()

    ros_image = bridge.cv2_to_imgmsg(camera_data, "bgr8")
    output_pub.publish(ros_image)

    # rospy.loginfo('Processing Data')

def intensity_to_color(intensity, min_intensity, max_intensity):
    # Normalize the intensity to the [0, 1] range
    norm_intensity = (intensity - min_intensity) / (max_intensity - min_intensity)
    # Convert normalized intensity to a color using the 'magma' colormap
    color = plt.get_cmap('hsv')(norm_intensity)  # This returns RGBA color
    # Convert RGBA to BGR color format used by OpenCV, and scale to [0, 255]
    return (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255))




def listener():
    rospy.init_node('syn_projection', anonymous=True)

    # image_sub = message_filters.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image)
    image_sub = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
    pcl_sub = message_filters.Subscriber('/hesai/pandar_points', PointCloud2)
        
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()









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






# def callback(image, pcl_data):
#     # global camera_data, lidar_data
#     # synchronized messages
#     camera_data = bridge.imgmsg_to_cv2(image, "bgr8")
#     lidar_data = np.array(list(pc2.read_points(pcl_data, skip_nans=True, field_names=('x', 'y', 'z',))))
    
#     filtered_data = voxel_downsampling(lidar_data)
#     lidar_points = np.hstack((filtered_data[:, :3], np.ones((filtered_data.shape[0], 1))))
#     lidar_points = np.transpose(lidar_points) 
#     lidar_points_cam = np.dot(extrinsic_matrix, lidar_points)  # lidar to camera coordinate

#     # projected point to image coordinate 
#     projected_points = np.dot(camera_matrix, lidar_points_cam[:3, :])
#     projected_points = np.transpose(projected_points)  
#     projected_points = projected_points[:, :2] / projected_points[:, 2:]

#     for i, point in enumerate(projected_points):
#         x, y = int(point[0]), int(point[1])
#         if 0 <= x < camera_data.shape[1] and 0 <= y < camera_data.shape[0]:
#             camera_data = cv2.circle(camera_data, (x, y), 2, (0, 255, 0), -1)


#     cv2.imshow('Camera with LiDAR', camera_data)
#     cv2.waitKey(1)