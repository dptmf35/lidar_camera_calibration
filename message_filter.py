import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from datetime import datetime
import os, cv2, numpy as np
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from pcl import PointCloud_PointXYZI
import pcl
from pcl import pcl_visualization

image_counter = 0
cloud_counter = 0
image_dir = "./data/distance_test"
cloud_dir = "./data/distance_test"
if not os.path.exists(image_dir):
    os.makedirs(image_dir)
if not os.path.exists(cloud_dir):
    os.makedirs(cloud_dir)

bridge = CvBridge()


def callback(image, pcl_data):
    global image_counter, cloud_counter
    # synchronized messages
    rospy.loginfo(f"Image Timestamp :{image.header.stamp} // PCL Timestamp : {pcl_data.header.stamp}")

    cvimg = bridge.imgmsg_to_cv2(image, "bgr8")
    cv2.imwrite(os.path.join(image_dir, f"{image_counter:06}.png"), cvimg)
    
    gen = pc2.read_points(pcl_data, skip_nans=True, field_names=("x", "y", "z", "intensity"))
    points = []
    for p in gen:
        # check point validation
        if p[0] == p[1] == p[2] == 0:
            continue
        else :
            points.append([p[0], p[1], p[2], p[3]])

    # pcl pointcloud() x, y , z only : no intensity 
    # p = pcl.PointCloud()
    # p.from_list(points) 
    p = PointCloud_PointXYZI()
    p.from_list(points)
    pcl.save(p, os.path.join(cloud_dir, f"{cloud_counter:06}.pcd"))

    image_counter+=1
    cloud_counter+=1
    rospy.loginfo(f"{image_counter}-th Image, {cloud_counter}-th PCD saved . . .")
    


def listener():
    rospy.init_node('my_synchronizer', anonymous=True)

    # image_sub = message_filters.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image)
    image_sub = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
    pcl_sub = message_filters.Subscriber('/hesai/pandar_points', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

# /zed2/zed_node/rgb_raw/image_raw_color
# /hesai/pandar_points

# class Subscriber:
#     def __init__(self):
#         self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", Image, self.callback1)
#         self.lidar_sub = rospy.Subscriber("/hesai/pandar_points", PointCloud2, self.callback2)

#     def callback1(self, data):
#         time= data.header.stamp.to_sec()
#         realtime = datetime.fromtimestamp(time).strftime('%Y년 %m월 %d일 %H시 %M분 %S초')
#         print("Image :",realtime)

#     def callback2(self,data) :
#         time= data.header.stamp.to_sec()
#         realtime = datetime.fromtimestamp(time).strftime('%Y년 %m월 %d일 %H시 %M분 %S초')
#         print("Lidar :", realtime)


# def main():
#     rospy.init_node('subscriber', anonymous=True)
#     subscriber = Subscriber()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
    

# if __name__ == '__main__':
#     main()
