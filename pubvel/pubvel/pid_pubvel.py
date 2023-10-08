import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Point, Twist
import math


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('pubvel_pid')
        self.get_state_client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.publisher_ = self.create_publisher(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.Kp = self.declare_parameter('Kp', 1.0).value
        self.Ki = self.declare_parameter('Ki', 0.1).value
        self.Kd = self.declare_parameter('Kd', 0.01).value
        self.get_logger().info(f"PID arams: Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}")
        
        self.min_vel = self.declare_parameter('min_vel', 3.0).value
        self.max_vel = self.declare_parameter('max_vel', 3.0).value

        desired_position_param = self.declare_parameter('desired_position', [4.0, 4.0, 0.0]).value
        self.desired_position = Point(x=desired_position_param[0], y=desired_position_param[1], z=desired_position_param[2])

        self.current_position = None
        self.prev_error = 0
        self.sum_error = 0

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def timer_callback(self):
        self.get_robot_position()
        
        if self.current_position:
            error_x = self.desired_position.x - self.current_position.x
            error_y = self.desired_position.y - self.current_position.y
            
            distance = math.sqrt(error_x**2 + error_y**2)
            
            if distance < 0.5: 
                self.get_logger().info("Reached the destination!")
                return
                
            # PID control for linear velocity
            self.sum_error += distance
            derivative_error = distance - self.prev_error
            linear_velocity = self.clamp(
                self.Kp * distance + self.Ki * self.sum_error + self.Kd * derivative_error,
                self.min_vel, 
                self.max_vel
            )                
                
            self.prev_error = distance

            # Compute angular velocity to face the desired position
            angle_to_target = math.atan2(error_y, error_x)
            angular_velocity = self.clamp(angle_to_target, self.min_vel, 2)

            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity
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

