import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Camera intrinsic parameter
camera_matrix = np.array([[263.16546630859375, 0.0, 321.3332824707031],
                           [0.0, 263.16546630859375, 188.22222900390625],
                           [0.0, 0.0, 1.0]], dtype=np.float32)


extrinsic_matrix = np.array(
[[-0.999649,-0.0248669,0.00912824,0.05193],
[-0.0102284,0.044481,-0.998957,0.131159],
[0.024435,-0.998701,-0.0447198,0.0918986],
[0,0,0,1]])

pcd_path = "./data/distance_test/000000.pcd"
image_path = "./data/distance_test/000000.png"


# 전역 변수
drawing = False # 마우스가 클릭된 상태인지 확인
ix, iy = -1, -1
box_coords = []

# 마우스 콜백 함수
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

def pc_to_cam(pcd_path, image_path, extrinsic_matrix, camera_matrix):
    pcd = o3d.io.read_point_cloud(pcd_path)
    image = cv2.imread(image_path)

    # Set up mouse callback to select box area in the image
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", draw_rectangle, image)

    while True:
        cv2.imshow("Image", image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27 or len(box_coords) == 4:  # ESC 키를 누르거나 박스 선택을 완료하면
            break
    cv2.destroyAllWindows()

    # Transform point cloud to camera coordinates
    pcd_points = np.asarray(pcd.points)
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
        print(f"Average distance within box: {average_distance} meters")
        return average_distance
    else:
        print("No points found within the selected box.")
        return None
    

pc_to_cam(pcd_path, image_path, extrinsic_matrix, camera_matrix)