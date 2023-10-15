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

# Define a function to convert a transformation matrix into a TransformStamped message
def transform_matrix_to_transform_stamped(matrix, parent_frame, child_frame):
    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame

    # Extract translation components from the matrix
    transform_stamped.transform.translation.x = matrix[0, 3]
    transform_stamped.transform.translation.y = matrix[1, 3]
    transform_stamped.transform.translation.z = matrix[2, 3]

    # Extract rotation components from the matrix
    rotation = np.zeros(4)
    rotation[3] = np.sqrt(max(0, 1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])) / 2
    rotation[0] = np.sqrt(max(0, 1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) / 2
    rotation[1] = np.sqrt(max(0, 1 - matrix[0, 0] + matrix[1, 1] - matrix[2, 2])) / 2
    rotation[2] = np.sqrt(max(0, 1 - matrix[0, 0] - matrix[1, 1] + matrix[2, 2])) / 2

    transform_stamped.transform.rotation.x = rotation[0]
    transform_stamped.transform.rotation.y = rotation[1]
    transform_stamped.transform.rotation.z = rotation[2]
    transform_stamped.transform.rotation.w = rotation[3]

    return transform_stamped


class TransformConcatenator(Node):

    def __init__(self):
        super().__init__('generate_map_frame')
        self.get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.get_ground_truth_async)  # 10Hz

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

                self.get_logger().info('map_H_baselink matrix:')
                self.get_logger().info('-----------------------')
                for row in map_H_baselink_matrix:
                    self.get_logger().info(f'| {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f} |')
                self.get_logger().info('-----------------------')

                #TODO -> DONE: odom_H_baselink from TransformStamped to matrix 
                odom_H_baselink = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                odom_H_baselink_matrix = calculate_transformation_matrix(
                    odom_H_baselink.transform.translation,
                    odom_H_baselink.transform.rotation
                )

                self.get_logger().info('odom_H_baselink matrix:')
                self.get_logger().info('-----------------------')
                for row in odom_H_baselink_matrix:
                    self.get_logger().info(f'| {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f} |')
                self.get_logger().info('-----------------------')

                #TODO -> DONE: map_H_baselink * inv(odom_H_baselink) = map_H_odom
                #TODO -> DONE: map_H_baselink * inv(odom_H_baselink) = map_H_odom
                map_H_odom_matrix = np.dot(map_H_baselink_matrix, np.linalg.inv(odom_H_baselink_matrix))


                self.get_logger().info('map_H_odom matrix:')
                self.get_logger().info('-----------------------')
                for row in map_H_odom_matrix:
                    self.get_logger().info(f'| {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f} |')
                self.get_logger().info('-----------------------')


                #TODO -> DONE: map_H_odom to TransformStamped
                map_H_odom_transform = transform_matrix_to_transform_stamped(
                    map_H_odom_matrix, 'map', 'odom')

                
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
        return transform_stamped

def main(args=None):
    rclpy.init(args=args)
    node = TransformConcatenator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        


