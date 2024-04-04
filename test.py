import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Camera intrinsic parameter
camera_matrix = np.array([[263.16546630859375, 0.0, 321.3332824707031],
                           [0.0, 263.16546630859375, 188.22222900390625],
                           [0.0, 0.0, 1.0]], dtype=np.float32)

# Lidar to camera extrinsic parameter
# gt matrix
# extrinsic_matrix = np.array([[-1.00000000e+00 , 1.83583740e-07, -3.66861455e-06 , 6.00003687e-02],
#                         [ 3.67320510e-06 , 4.99791693e-02, -9.98750260e-01,  1.00387282e-01],
#                         [ 5.69184965e-18 ,-9.98750260e-01, -4.99791693e-02 , 1.12299792e-01],
#                         [0, 0, 0, 1]])

extrinsic_matrix = np.array(  [[-1.00000000e+00 , 1.83583740e-07, -3.66861455e-06,  6.00003687e-02],
 [ 3.67320510e-06,  4.99791693e-02, -9.98750260e-01,  1.00387282e-01],
 [ 5.69184965e-18, -9.98750260e-01, -4.99791693e-02,  1.37299792e-01],
 [0, 0, 0, 1]])

# tf matrix 0403

extrinsic_matrix = np.array( [[-0.999789,-0.0205286,-6.02334e-05,0.0660872],
[-0.000751974,0.0395547,-0.999217,0.115046],
[0.0205149,-0.999007,-0.0395618,0.0938102],
[0,0,0,1]])

# 0403 8:54 pm


extrinsic_matrix = np.array( 
[[-0.999636,-0.0243892,0.0115024,0.0501407],
[-0.0126799,0.0486895,-0.998733,0.144388],
[0.0237983,-0.998516,-0.0489811,0.0865961],
[0,0,0,1]])

# 0404 10 am

extrinsic_matrix = np.array(
[[-0.999649,-0.0248669,0.00912824,0.05193],
[-0.0102284,0.044481,-0.998957,0.131159],
[0.024435,-0.998701,-0.0447198,0.0918986],
[0,0,0,1]])



# ground truth rpy, rot  -------------------
# rpy = np.array([0.4759126427027314, 0.15494010098291655, -0.5862306408466508])
# rotation_matrix = np.array([[ -1.00000000e+00 , 1.83583740e-07, -3.66861455e-06],
# [3.67320510e-06,  4.99791693e-02, -9.98750260e-01],
# [ 5.69184965e-18, -9.98750260e-01, -4.99791693e-02]]
# )
# translation = np.array([ 6.00003687e-02, 1.00387282e-01, 1.37299792e-01])

# ------------------- -------------------  -------------------

# rpy = np.array([-1.6166027543190002, -0.018972376796441516, 3.091103175440616])
# rotation_matrix = np.array( [[-0.99854594, -0.01661624, -0.05128269],
# [ 0.05045895,  0.0466885,  -0.99763424],
# [ 0.01897124, -0.99877129, -0.04578217]]
# )
# translation = np.array([ 0.061231545, 0.12366614, 0.04249328])

# ------------------- -------------------  -------------------

# rpy = np.array([-1.556898839059469, -0.048599870562104186, -3.109222283107868])
# rotation_matrix = np.array([[-0.998296,   -0.04810083,  0.03303637],
# [-0.0323265,  -0.01546191, -0.99935776],
# [ 0.04858074, -0.9987228,   0.01388063]]
# )
# translation = np.array([ 6.00003687e-02, 1.00387282e-01, 1.37299792e-01])

# #

# tf_matrix = np.eye(4)
# tf_matrix[:3, :3] = rotation_matrix
# tf_matrix[:3, 3] = translation

# extrinsic_matrix = tf_matrix.copy()


def pc_to_cam(pcd_path, image_path) : 
    pcd = o3d.io.read_point_cloud(pcd_path)

    image = cv2.imread(image_path)
    origin_img = image.copy()



    # Transform point cloud to camera coordinates
    pcd_points = np.asarray(pcd.points)
    pcd_points_homogeneous = np.hstack((pcd_points, np.ones((pcd_points.shape[0], 1))))
    pcd_points_camera = np.dot(pcd_points_homogeneous, extrinsic_matrix.T)[:, :3].astype(np.float32)  # Ensure data type is float32

    # Project points onto image
    rvec = np.zeros((3,), dtype=np.float32)
    tvec = np.zeros((3,), dtype=np.float32)
    projected_points, _ = cv2.projectPoints(pcd_points_camera, rvec, tvec, camera_matrix, None)


    # Assign colors based on distance
    distances = np.linalg.norm(pcd_points, axis=1)
    x_normalized = (distances - np.min(distances)) / (np.max(distances) - np.min(distances))
    colors = plt.cm.hsv(x_normalized)[:, :3]
    # print(colors)

    # Draw projected points on image
    # for p in projected_points:
    for p, color in zip(projected_points, colors) :
        x, y = int(p[0][0]), int(p[0][1])
        # cv2.circle(image, (x, y), 1, (0, 255, 0), -1)  # Green color for projected points
        # color = pcd.colors[y, x] if y < pcd.colors.shape[0] and x < pcd.colors.shape[1] else (0, 0, 255)  # Use red color if point cloud color is not available
        # cv2.circle(image, (x, y), 1, color, -1)  # Use color from point cloud for the circle
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (x, y), 1, (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255)), -1)

    return image, origin_img

# def voxel_grid_filter(points, voxel_size):
#         grid = {}

#         # Grouping points into voxels
#         for point in points:
#             voxel = tuple(int(round(coord / voxel_size)) for coord in point[:-1])  # Exclude intensity for voxel calculation
#             grid.setdefault(voxel, []).append(point)

#         # Keeping only one point per voxel
#         downsampled_points = []
#         for voxel_points in grid.values():
#             # Calculate the mean point for each voxel
#             voxel_sum = [sum(coords) for coords in zip(*voxel_points)]
#             voxel_mean = [coord / len(voxel_points) for coord in voxel_sum]
#             downsampled_points.append(voxel_mean)

#         return downsampled_points    


# # Load point cloud from PCD file
# pcd_path = "./data/test_pc/000001.pcd"
# pcd = o3d.io.read_point_cloud(pcd_path)

# # Load image
# image_path = "./data/test_image/000001.png"
# image = cv2.imread(image_path)
# origin_img = image.copy()


# # Transform point cloud to camera coordinates
# pcd_points = np.asarray(pcd.points)
# pcd_points_homogeneous = np.hstack((pcd_points, np.ones((pcd_points.shape[0], 1))))
# pcd_points_camera = np.dot(pcd_points_homogeneous, extrinsic_matrix.T)[:, :3].astype(np.float32)  # Ensure data type is float32

# # Project points onto image
# rvec = np.zeros((3,), dtype=np.float32)
# tvec = np.zeros((3,), dtype=np.float32)
# projected_points, _ = cv2.projectPoints(pcd_points_camera, rvec, tvec, camera_matrix, None)


# # Assign colors based on distance
# distances = np.linalg.norm(pcd_points, axis=1)
# x_normalized = (distances - np.min(distances)) / (np.max(distances) - np.min(distances))
# colors = plt.cm.hsv(x_normalized)[:, :3]
# # print(colors)

# # Draw projected points on image
# # for p in projected_points:
# for p, color in zip(projected_points, colors) :
#     x, y = int(p[0][0]), int(p[0][1])
#     # cv2.circle(image, (x, y), 1, (0, 255, 0), -1)  # Green color for projected points
#     # color = pcd.colors[y, x] if y < pcd.colors.shape[0] and x < pcd.colors.shape[1] else (0, 0, 255)  # Use red color if point cloud color is not available
#     # cv2.circle(image, (x, y), 1, color, -1)  # Use color from point cloud for the circle
#     if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
#         cv2.circle(image, (x, y), 1, (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255)), -1)


# # Display image with projected points
# cv2.imshow("Projected LiDAR Points", cv2.hconcat([image, origin_img]))
# cv2.waitKey(0)
# cv2.destroyAllWindows()
    
pcd_num = 0
img_num = 0

# Load point cloud from PCD, image file



for i in range(70) :
    pcd_path = f"./data/test_pc/{pcd_num:06}.pcd"
    image_path = f"./data/test_image/{img_num:06}.png"

    image, origin_img = pc_to_cam(pcd_path,image_path)
    print(pcd_path, image_path)
    # Display image with projected points
    cv2.imshow("Projected LiDAR Points", cv2.hconcat([image, origin_img]))
    cv2.imwrite(f"./data/test_image_projected/{img_num:06}.png",image)
    pcd_num +=1
    img_num +=1
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
cv2.destroyAllWindows()
