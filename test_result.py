import numpy as np

# Given Euler angles (RPY)
roll, pitch, yaw = 0.08172671931808902, -0.018876674379351983, -0.13643777438877575


# Euler angles (RPY): (-0.14451271106509783, -0.19746716474893472, 0.12199261122443718)
# Rotation Matrix: [[ 0.97327918 -0.0923789  -0.21022315]
#  [ 0.1193254   0.98565989  0.11931486]
#  [ 0.19618635 -0.14121163  0.9703454 ]]
# Translation Offsets: [[0. 0. 0.]]


# Given rotation matrix
R = np.array([[0.99053029, 0.13403428, -0.0297414],
              [-0.13599063, 0.98760963, -0.07831833],
              [0.01887555, 0.08162123, 0.99648467]])


# Given translation offsets
translation = np.array([[1.45199236e+21, 3.85201139e+27, 9.54439421e+26]]).T


# Create a rotation matrix from Euler angles
def euler_to_rotation_matrix(roll, pitch, yaw):
    # Conversion from Euler angles to rotation matrix
    # R_x = | 1      0           0      |
    #       | 0   cos(roll)  -sin(roll) |
    #       | 0   sin(roll)   cos(roll) |
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    # R_y = | cos(pitch)   0   sin(pitch) |
    #       |      0        1       0      |
    #       |-sin(pitch)   0   cos(pitch) |
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    # R_z = | cos(yaw)  -sin(yaw)   0   |
    #       | sin(yaw)   cos(yaw)   0   |
    #       |    0           0       1   |
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combined rotation matrix
    R_combined = np.dot(R_z, np.dot(R_y, R_x))
    return R_combined

# Create rotation matrix from Euler angles
R_from_euler = euler_to_rotation_matrix(roll, pitch, yaw)

# Create transformation matrix
T = np.hstack((R, translation))
transform_matrix = np.vstack((T, [0, 0, 0, 1]))

print("Transformation Matrix:")
print(transform_matrix)