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
# * PECORE - Master en Automática y Control en Robótica                *
# * Universitat Politècnica de Catalunya (UPC)                         *
# *                                                                    *
# * Participantes:                                                     *
# * - Victor Escribano Garcia                                          *
# * - Alejandro Acosta Montilla                                        *
# *                                                                    *
# * Año: 2023                                                                                       
"""
Descripcion: Codigo encargado de obtener la transformacion de la pose deseada en base al mapa y encargado de seguir ese objetivo.
"""                                                                                  

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform
from tf2_ros import TransformListener, Buffer
import numpy as np
from visual_servoing_P1.transformations  import homo_matrix2tf, Rt2homo_matrix, transform2pose

class ArUcoApproachNode(Node):
    def __init__(self):
        super().__init__('aruco_approach')
        
        # Subscriber to ArUco marker pose
        self.subscription_aruco = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            10)
        self.subscription_aruco  # prevent unused variable warning
        
        # Publisher for robot goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Desired camera pose w.r.t. the target (ArUco marker)
        self.aruco_H_desPos = TransformStamped()
        self.aruco_H_desPos.transform.translation.x = 0.0
        self.aruco_H_desPos.transform.translation.y = 0.0
        self.aruco_H_desPos.transform.translation.z = 0.1
        self.aruco_H_desPos.transform.rotation.x = -0.707
        self.aruco_H_desPos.transform.rotation.y = 0.707
        self.aruco_H_desPos.transform.rotation.z = 0.0
        self.aruco_H_desPos.transform.rotation.w = 0.707
        self.aruco_H_desPos.header.frame_id = 'aruco_marker_frame'
        self.aruco_H_desPos.child_frame_id = 'desired_pose'
        self.aruco_H_desPos.header.stamp = self.get_clock().now().to_msg()

        
    def aruco_callback(self, msg):
        try:

            # Define the transformation between the camera and aruco frame
            cam_H_aruco = TransformStamped()
            cam_H_aruco.transform.translation.x = msg.pose.position.x
            cam_H_aruco.transform.translation.y = msg.pose.position.y
            cam_H_aruco.transform.translation.z = msg.pose.position.z
            cam_H_aruco.transform.rotation = msg.pose.orientation

            # Look up the required transformations
            now = rclpy.time.Time()
            # cam_H_aruco = self.tf_buffer.lookup_transform('front_camera_mount', 'aruco_marker_frame', now)
            robot_H_cam = self.tf_buffer.lookup_transform('base_link', 'front_camera_optical', now)
            map_H_robot = self.tf_buffer.lookup_transform('map', 'base_link', now)

            cam_H_aruco_matrix = Rt2homo_matrix(cam_H_aruco.transform.translation, cam_H_aruco.transform.rotation)
            robot_H_cam_matrix = Rt2homo_matrix(robot_H_cam.transform.translation, robot_H_cam.transform.rotation)
            map_H_robot_matrix = Rt2homo_matrix(map_H_robot.transform.translation, map_H_robot.transform.rotation)
            aruco_H_desPos_matrix = Rt2homo_matrix(self.aruco_H_desPos.transform.translation, self.aruco_H_desPos.transform.rotation)

            # Compute the desired robot pose in the map frame
            map_H_desrobot_matrix = np.dot(map_H_robot_matrix, np.dot(robot_H_cam_matrix, np.dot(cam_H_aruco_matrix, aruco_H_desPos_matrix)))

            # Convert the result to a TransformStamped
            map_H_desPos_transform = homo_matrix2tf(map_H_desrobot_matrix, 'map', 'desired_pose', self.get_clock().now().to_msg())
            
            ## Convert the result to a PoseStamped message and publish it
            goal_pose = transform2pose(map_H_desPos_transform)
            self.get_logger().info('Pose comuted: ' + str(goal_pose))
            self.goal_publisher.publish(goal_pose)

        except Exception as e:
            self.get_logger().info('Failed to compute goal pose: ' + str(e))
            

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()