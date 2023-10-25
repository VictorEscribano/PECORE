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
Descripcion: Codigo encargado de obtener la transformacion entre el sistema de referencia de la base del robot y el sistema de referencia del mapa.s
"""                                                                        


import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from visual_servoing_P1.transformations  import homo_matrix2tf, Rt2homo_matrix



class TransformConcatenator(Node):

    def __init__(self):
        super().__init__('generate_map_frame')
        self.get_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.get_ground_truth_async)  # 10Hz
        
        # Generate dummy subscription to have always the aruco marker frame on the tf tree in case aruco_follower.py is not used
        self.subscription_aruco = self.create_subscription(PoseStamped,'/aruco_single/pose',self.aruco_callback,10) 
        self.subscription_aruco  # prevent unused variable warning
    
    def get_ground_truth_async(self):
        ## Wait for the service to be available
        self.get_state_client.wait_for_service()
        
        request = GetEntityState.Request()
        request.name = "jackal"
        
        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.handle_get_entity_state_response)


    def handle_get_entity_state_response(self, future):
        try:
            response = future.result()
            if response:
                #map_H_baselink from TransformStamped to matrix 
                map_H_baselink_matrix = Rt2homo_matrix(response.state.pose.position, response.state.pose.orientation)
        
                #odom_H_baselink from TransformStamped to matrix 
                odom_H_baselink = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                odom_H_baselink_matrix = Rt2homo_matrix(odom_H_baselink.transform.translation, odom_H_baselink.transform.rotation)
                
                # map_H_baselink * inv(odom_H_baselink) = map_H_odom
                map_H_odom_matrix = np.dot(map_H_baselink_matrix, np.linalg.inv(odom_H_baselink_matrix))
                
                #map_H_odom to TransformStamped
                map_H_odom_transform = homo_matrix2tf(map_H_odom_matrix, 'map', 'odom', self.get_clock().now().to_msg())

                #publish map_H_odom
                self.tf_broadcaster.sendTransform(map_H_odom_transform)
  
            else:
                self.get_logger().error('No response from service.')
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')

    # Dummy callback function for the subscription to aruco pose
    def aruco_callback(self, msg):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = TransformConcatenator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()