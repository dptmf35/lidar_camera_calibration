import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from datetime import datetime
import os, cv2, numpy as np
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

camera_matrix = np.array([[263.16546630859375, 0.0, 321.3332824707031],
                           [0.0, 263.16546630859375, 188.22222900390625],
                           [0.0, 0.0, 1.0]], dtype=np.float32)


extrinsic_matrix = np.array(
[[-0.999649,-0.0248669,0.00912824,0.05193],
[-0.0102284,0.044481,-0.998957,0.131159],
[0.024435,-0.998701,-0.0447198,0.0918986],
[0,0,0,1]])


bridge = CvBridge()


drawing = False 
ix, iy = -1, -1
box_coords = []
print_info = True

# mouse callback
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, box_coords

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            img_copy = param.copy()
            cv2.rectangle(img_copy, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow("Image", img_copy)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        box_coords = [ix, iy, x, y]
        cv2.rectangle(param, (ix, iy), (x, y), (0, 255, 0), 2)
        cv2.imshow("Image", param)


def callback(image_data, pcl_data, depth_data):
    global box_coords, print_info

    # synchronized messages
    cvimg = bridge.imgmsg_to_cv2(image_data, "bgr8")
    points = pc2.read_points(pcl_data, skip_nans=True, field_names=("x", "y", "z", "intensity"))

    # Set up mouse callback to select box area in the image
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", draw_rectangle, cvimg)
    if print_info :
        rospy.loginfo("[Drag] to select bounding box area [Enter] to find new bounding box [ESC] or [Q] to end process") 
        print_info = False

    while True:
        cv2.imshow("Image", cvimg)
        k = cv2.waitKey(1) & 0xFF

        if k == 27 or k == ord('q'):  # ESC or q key 
            cv2.destroyAllWindows()
            rospy.signal_shutdown('User requested shutdown')  # exit ROS node
            break

        if k == 13: # reset bbox area
            box_coords = []   
            print("Select new bounding box")

        if len(box_coords) == 4:  # set bbox area
            break



    pcd_points = np.array(list(points))[:, :3]
    pcd_points_homogeneous = np.hstack((pcd_points, np.ones((pcd_points.shape[0], 1))))
    pcd_points_camera = np.dot(pcd_points_homogeneous, extrinsic_matrix.T)[:, :3].astype(np.float32)

    # Project points onto image
    rvec = np.zeros((3,), dtype=np.float32)
    tvec = np.zeros((3,), dtype=np.float32)
    projected_points, _ = cv2.projectPoints(pcd_points_camera, rvec, tvec, camera_matrix, None)

    # Calculate average distance of points within the selected box
    distances = []
    x_min, y_min, x_max, y_max = box_coords
    for p, pc_point in zip(projected_points, pcd_points_camera):
        x, y = int(p[0][0]), int(p[0][1])
        if x_min <= x <= x_max and y_min <= y <= y_max:
            distances.append(np.linalg.norm(pc_point))

    if distances:
        average_distance = np.mean(distances)
    else:
        average_distance = None

    depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")

    depth_values = depth_image[y_min:y_max, x_min:x_max]
    valid_depths = depth_values[np.isfinite(depth_values)]
    if valid_depths.size > 0:
        average_depth = np.mean(valid_depths)
    else:
        average_depth = None
    rospy.loginfo(f"Average distance within box: {average_distance} meters // Average depth in bbox: {average_depth:.3f} meters")


def listener():
    rospy.init_node('my_synchronizer', anonymous=True)

    image_sub = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image)
    pcl_sub = message_filters.Subscriber('/hesai/pandar_points', PointCloud2)
    depth_sub = message_filters.Subscriber("/zed2/zed_node/depth/depth_registered", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub, depth_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
