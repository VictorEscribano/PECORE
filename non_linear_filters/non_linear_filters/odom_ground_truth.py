import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry

class odom_ground_truth(Node):
    def __init__(self, entity_name='jackal', timer_period=0.5):
        super().__init__('odom_ground_truth')
        self.entity_name = entity_name
        self.timer_period = timer_period
        self.initialize_service_clients()
        self.initialize_publishers()
        self.initialize_timer()

    def initialize_service_clients(self):
        self.get_state_client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.get_logger().info('Waiting for service...')
        self.get_state_client.wait_for_service()

    def initialize_publishers(self):
        self.odom_publisher_ = self.create_publisher(Odometry, '/odometry/ground_truth', 10)

    def initialize_timer(self):
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.current_position = None

    def timer_callback(self):
        self.get_robot_position()

    def get_robot_position(self):
        request = GetEntityState.Request()
        request.name = self.entity_name
        request.reference_frame = "world"
        
        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.handle_get_entity_state_response)

    def handle_get_entity_state_response(self, future):
        try:
            response = future.result()
            if response:
                self.publish_odometry(response)
            else:
                self.get_logger().error('No response from service.')
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')

    def publish_odometry(self, response):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose = response.state.pose
        self.odom_publisher_.publish(odom)
        self.get_logger().info(f'pose: {odom.pose.pose}')

def main(args=None):
    rclpy.init(args=args)
    entity_name = 'jackal'  # Este valor podría ser parametrizado
    node = odom_ground_truth(entity_name=entity_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
