import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

drawing = False  # 마우스가 클릭된 상태인지 확인
ix, iy = -1, -1
bbox = []

def select_bbox(event, x, y, flags, param):
    global ix, iy, drawing, bbox

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            img_copy = param.copy()
            cv2.rectangle(img_copy, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow("Color Image", img_copy)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        bbox = [ix, iy, x, y]
        cv2.rectangle(param, (ix, iy), (x, y), (0, 255, 0), 2)
        cv2.imshow("Color Image", param)

class ZEDCameraViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.color_callback)
        self.depth_image = None
        rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depth_callback)
        rospy.loginfo("ZED Camera Viewer has started.")

    def color_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.namedWindow("Color Image")
            cv2.setMouseCallback("Color Image", select_bbox, cv_image)

            while True:
                cv2.imshow("Color Image", cv_image)
                k = cv2.waitKey(1) & 0xFF
                if k == 27 or bbox:  # ESC 키를 누르거나 박스 선택을 완료하면
                    break

            if bbox and self.depth_image is not None:
                self.calculate_average_depth()

            cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def calculate_average_depth(self):
        x_min, y_min, x_max, y_max = bbox
        depth_values = self.depth_image[y_min:y_max, x_min:x_max]
        # print(self.depth_image)
        valid_depths = depth_values[np.isfinite(depth_values)]
        if valid_depths.size > 0:
            average_depth = np.mean(valid_depths)
            rospy.loginfo(f"Average depth in bbox: {average_depth:.3f} meters")
        else:
            rospy.loginfo("No valid depth data in the specified bbox.")

if __name__ == "__main__":
    rospy.init_node("zed_camera_viewer", anonymous=True)
    viewer = ZEDCameraViewer()
    rospy.spin()



