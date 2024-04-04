import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from itertools import islice
import numpy as np

class PointCloudDownsampler:
    def __init__(self):
        rospy.init_node('point_cloud_downsampler', anonymous=True)
        
        # Subscriber for the input point cloud
        rospy.Subscriber('/hesai/pandar_points', PointCloud2, self.point_cloud_callback)
        
        # Publisher for the downsampled point cloud
        self.pub = rospy.Publisher('/hesai/new_points', PointCloud2, queue_size=10)

    def point_cloud_callback(self, msg):
        # Downsample the point cloud
        downsampled_msg = self.downsample_point_cloud(msg, 0.025)
        
        # Publish the downsampled point cloud
        self.pub.publish(downsampled_msg)

    def downsample_point_cloud(self, msg, voxel_size):
        points = pc2.read_points(msg, field_names=("x", "y", "z","intensity"), skip_nans=True)
        downsampled_points = self.voxel_grid_filter(points, voxel_size)
        intensities = [point[3] for point in points]

        # Creating a new PointCloud2 message with downsampled points
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        # Create a list of tuples containing (x, y, z, intensity)
        downsampled_point_tuples = [(point[0], point[1], point[2], point[3]) for point in downsampled_points]

        # Create PointCloud2 message
        downsampled_msg = pc2.create_cloud(header, fields=[pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                                                        pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                                                        pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                                                        pc2.PointField(name="intensity", offset=12, datatype=pc2.PointField.FLOAT32, count=1)],
                                        points=downsampled_point_tuples)

        return downsampled_msg
    

    def voxel_grid_filter(self, points, voxel_size):
        grid = {}

        # Grouping points into voxels
        for point in points:
            voxel = tuple(int(round(coord / voxel_size)) for coord in point[:-1])  # Exclude intensity for voxel calculation
            grid.setdefault(voxel, []).append(point)

        # Keeping only one point per voxel
        downsampled_points = []
        for voxel_points in grid.values():
            # Calculate the mean point for each voxel
            voxel_sum = [sum(coords) for coords in zip(*voxel_points)]
            voxel_mean = [coord / len(voxel_points) for coord in voxel_sum]
            downsampled_points.append(voxel_mean)

        return downsampled_points

if __name__ == '__main__':
    try:
        downsampler = PointCloudDownsampler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass