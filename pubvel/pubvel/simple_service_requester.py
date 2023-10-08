import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState


class SimpleServiceRequester(Node):
    def __init__(self):
        super().__init__('simple_service_requester')
        self.get_state_client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        
        # Once the node is created, we'll make the service call.
        self.make_service_call()

    def make_service_call(self):
        # Wait for the service to be available
        self.get_logger().info('Waiting for service...')
        self.get_state_client.wait_for_service()
        
        # Create a request
        request = GetEntityState.Request()
        request.name = "jackal"
        request.reference_frame = "world"
        
        # Call the service
        self.get_logger().info('Calling service...')
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()

        # Print the response
        if response:
            self.get_logger().info(f'Jackal Position: x={response.state.pose.position.x}, y={response.state.pose.position.y}, z={response.state.pose.position.z}')
        else:
            self.get_logger().error('No response from service.')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceRequester()
    rclpy.spin(node)  # This will keep the node alive so the service call can be processed.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

