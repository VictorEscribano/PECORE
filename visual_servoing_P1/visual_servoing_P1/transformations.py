""" 
$$$$$$$\  $$$$$$$$\  $$$$$$\   $$$$$$\  $$$$$$$\  $$$$$$$$\       $$$$$$$\    $$\   
$$  __$$\ $$  _____|$$  __$$\ $$  __$$\ $$  __$$\ $$  _____|      $$  __$$\ $$$$ |  
$$ |  $$ |$$ |      $$ /  \__|$$ /  $$ |$$ |  $$ |$$ |            $$ |  $$ |\_$$ |  
$$$$$$$  |$$$$$\    $$ |      $$ |  $$ |$$$$$$$  |$$$$$\          $$$$$$$  |  $$ |  
$$  ____/ $$  __|   $$ |      $$ |  $$ |$$  __$$< $$  __|         $$  ____/   $$ |  
$$ |      $$ |      $$ |  $$\ $$ |  $$ |$$ |  $$ |$$ |            $$ |        $$ |  
$$ |      $$$$$$$$\ \$$$$$$  | $$$$$$  |$$ |  $$ |$$$$$$$$\       $$ |      $$$$$$\ 
\__|      \________| \______/  \______/ \__|  \__|\________|      \__|      \______|
"""
# * PECORE - Master en Automática y Control en Robótica               *
# * Universitat Politècnica de Catalunya (UPC)                         *
# *                                                                    *
# * Participantes:                                                     *
# * - Victor Escribano Garcia                                          *
# * - Alejandro Acosta Montilla                                       *
# *                                                                    *
# * Año: 2023                                                                                       
"""
Descripcion: Funciones para poder transformar entre matrices de transformacion y TransformStamped de ROS. 
"""                                                                        

from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np

# Define a function to calculate the transformation matrix
def Rt2homo_matrix(translation, rotation):
    # Reconstruct the transformation matrix
    # Create a 4x4 identity matrix
    matrix = np.identity(4)

    # Set the translation components in the matrix
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z

    # Extract the rotation components
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w

    # Calculate the rotation matrix components
    xx = x * x
    xy = x * y
    xz = x * z
    yy = y * y
    yz = y * z
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z

    # Set the rotation components in the matrix
    matrix[0, 0] = 1 - 2 * (yy + zz)
    matrix[0, 1] = 2 * (xy - wz)
    matrix[0, 2] = 2 * (xz + wy)

    matrix[1, 0] = 2 * (xy + wz)
    matrix[1, 1] = 1 - 2 * (xx + zz)
    matrix[1, 2] = 2 * (yz - wx)

    matrix[2, 0] = 2 * (xz - wy)
    matrix[2, 1] = 2 * (yz + wx)
    matrix[2, 2] = 1 - 2 * (xx + yy)

    return matrix


def homo_matrix2tf(matrix, parent_frame, child_frame, clock_stamp):
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = clock_stamp
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame

    # Extract translation from the matrix
    translation = np.array([matrix[0, 3], matrix[1, 3], matrix[2, 3]])

    # Extract rotation (quaternion) from the matrix
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s

    # Set the translation and rotation in the TransformStamped message
    transform_stamped.transform.translation.x = translation[0]
    transform_stamped.transform.translation.y = translation[1]
    transform_stamped.transform.translation.z = translation[2]
    transform_stamped.transform.rotation.x = x
    transform_stamped.transform.rotation.y = y
    transform_stamped.transform.rotation.z = z
    transform_stamped.transform.rotation.w = w

    return transform_stamped

def transform2pose(transform):
    pose = PoseStamped()
    pose.header = transform.header
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose