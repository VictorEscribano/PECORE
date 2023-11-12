import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np

# Define a function to calculate the transformation matrix
def calculate_transformation_matrix(translation, rotation):
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



class TransformConcatenator(Node):

    def __init__(self):
        super().__init__('generate_map_frame')
        self.get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.get_ground_truth_async)  # 10Hz


    def transform_matrix_to_transform_stamped(self, matrix, parent_frame, child_frame):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
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



    def get_ground_truth_async(self):
        ## Wait for the service to be available
        self.get_logger().info('Waiting for service...')
        self.get_state_client.wait_for_service()
        
        request = GetEntityState.Request()
        request.name = "jackal"
        
        self.get_logger().info('Calling service...')
        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.handle_get_entity_state_response)

    def handle_get_entity_state_response(self, future):
        self.get_logger().info('Service response received.')
        
        
        
        try:
            response = future.result()
            if response:
                #TODO -> DONE: map_H_baselink from TransformStamped to matrix 
                map_H_baselink_matrix = calculate_transformation_matrix(
                    response.state.pose.position,
                    response.state.pose.orientation
                )

                #TODO -> DONE: odom_H_baselink from TransformStamped to matrix 
                odom_H_baselink = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                odom_H_baselink_matrix = calculate_transformation_matrix(
                    odom_H_baselink.transform.translation,
                    odom_H_baselink.transform.rotation
                )

                #TODO -> DONE: map_H_baselink * inv(odom_H_baselink) = map_H_odom
                #TODO -> DONE: map_H_baselink * inv(odom_H_baselink) = map_H_odom
                map_H_odom_matrix = np.dot(map_H_baselink_matrix, np.linalg.inv(odom_H_baselink_matrix))

                #TODO -> DONE: map_H_odom to TransformStamped
                map_H_odom_transform = self.transform_matrix_to_transform_stamped(map_H_odom_matrix, 'map', 'odom')
                
                #TODO -> DONE: publish map_H_odom
                self.tf_broadcaster.sendTransform(map_H_odom_transform)

                
            else:
                self.get_logger().error('No response from service.')
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')

    def pose_to_transform_stamped(self, pose, parent_frame, child_frame):
        transform_stamped = TransformStamped()
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation = pose.orientation
        transform_stamped.header.frame_id = parent_frame
        transform_stamped.child_frame_id = child_frame
        #stamp now
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        return transform_stamped

def main(args=None):
    rclpy.init(args=args)
    node = TransformConcatenator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()