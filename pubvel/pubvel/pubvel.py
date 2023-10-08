import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Point, Twist
import math


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('pubvel')
        self.get_state_client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.publisher_ = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_position = None  


    def timer_callback(self):
        desired_position = Point(x=4.0, y=0.0, z=0.0)  # Hardcoded desired position
        self.get_robot_position()
        
        if self.current_position:
            error_x = desired_position.x - self.current_position.x
            error_y = desired_position.y - self.current_position.y
            
            distance = math.sqrt(error_x**2 + error_y**2)
            
            if distance < 0.1: 
                self.get_logger().info("Reached the destination!")
                return
                
            unit_x = error_x / distance
            unit_y = error_y / distance
            
            # Compute linear and angular velocities
            linear_velocity = unit_x  # Assuming same magnitude for simplicity
            angular_velocity = math.atan2(unit_y, unit_x)
            
            msg = Twist()
            msg.linear.x = linear_velocity  # Change this value as desired
            msg.angular.z = angular_velocity  # Change this value as desired
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing velocity')
            
        
        
    def get_robot_position(self):
    	## Wait for the service to be available
        self.get_logger().info('Waiting for service...')
        self.get_state_client.wait_for_service()
        
        # Create a request
        request = GetEntityState.Request()
        request.name = "jackal"
        request.reference_frame = "world"
        
        # Call the service
        self.get_logger().info('Calling service...')
        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.handle_get_entity_state_response)



    def handle_get_entity_state_response(self, future):
        try:
            response = future.result()
            if response:
                self.current_position = response.state.pose.position  # Almacenar la posición en la variable de instancia
                self.get_logger().info(f'Jackal Position: x={self.current_position.x}, y={self.current_position.y}, z={self.current_position.z}')
            else:
                self.get_logger().error('No response from service.')
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

