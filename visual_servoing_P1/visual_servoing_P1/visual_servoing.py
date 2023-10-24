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
Descripcion: Codigo encargado de enviar comandos de velocidad basados en el visual servoing.
"""  


import rclpy
from rclpy.node import Node

import numpy as np
from visual_servoing_P1.transformations  import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped

class VisualServoing(Node):

    def __init__(self):

        super().__init__('visual_servoing')

        # Create a subscription to the pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Definition of all necessary callback groups
        self.pub_group=MutuallyExclusiveCallbackGroup()		
        timer_period= 0.1																							# Default timer seconds to publish a new command for pose or speed
        self.timer= self.create_timer(timer_period, self.pose_callback, callback_group=self.pub_group)

        # Set up a publisher for controlling the jackal
        self.publisher = self.create_publisher(Twist, 'jackal_velocity_controller/cmd_vel_unstamped', 10)

        # Define the transformation between the aruco and descam frame
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

        self.cam_H_aruco = TransformStamped()

        self.new_pose_received = False
        
    def aruco_callback(self, msg):
        self.cam_H_aruco.transform.translation.x = msg.pose.position.x
        self.cam_H_aruco.transform.translation.y = msg.pose.position.y
        self.cam_H_aruco.transform.translation.z = msg.pose.position.z
        self.cam_H_aruco.transform.rotation = msg.pose.orientation

        self.new_pose_received = True

    def pose_callback(self):
        try:
            if self.new_pose_received:
                # Convert the Transform messages to numpy matrices
                cam_H_aruco_matrix = Rt2homo_matrix(self.cam_H_aruco.transform.translation, self.cam_H_aruco.transform.rotation)
                aruco_H_desPos_matrix = Rt2homo_matrix(self.aruco_H_desPos.transform.translation, self.aruco_H_desPos.transform.rotation)

                # Compute the resultant transformation by multiplying the two matrices
                cam_H_descam_mat = np.dot(cam_H_aruco_matrix,aruco_H_desPos_matrix)

                # Convert the resulting numpy matrix back to a ROS message
                cam_H_descam = homo_matrix2tf(cam_H_descam_mat, 'front_camera_optical', 'desired_pose', self.get_clock().now().to_msg())

                # Compute the Jacobian for the system
                J = np.identity(6)
                J_p = cam_H_descam_mat[:3,:3]
                J[:3,:3] = J_p
                J_r = 0.5 * (np.trace(np.transpose(J_p)) * np.identity(3) - J_p)
                J[3:6,3:6] = J_r

                # Calculate the error in position and orientation
                e_p = np.array([cam_H_descam.transform.translation.x, cam_H_descam.transform.translation.y, cam_H_descam.transform.translation.z])
                q = [cam_H_descam.transform.rotation.x, cam_H_descam.transform.rotation.y, cam_H_descam.transform.rotation.z, cam_H_descam.transform.rotation.w]
                e_phi = list(quaternion2euler(cam_H_descam.transform.rotation.x, cam_H_descam.transform.rotation.y, cam_H_descam.transform.rotation.z, cam_H_descam.transform.rotation.w))
                e = np.concatenate([e_p,e_phi])

                # Calculate the velocity using the visual servoing control law
                labd_val = 1
                v = labd_val * np.dot(np.linalg.pinv(J),np.transpose(e))

                max_vel_trans = 1.5
                if v[0] > max_vel_trans:
                    v[0] = max_vel_trans
                elif v[0] < -max_vel_trans:
                    v[0] = -max_vel_trans
                    
                max_vel_rot = 0.6
                if v[5] > max_vel_rot:
                    v[5] = max_vel_rot
                elif v[5] < -max_vel_rot:
                    v[5] = -max_vel_rot

                # Populate the Twist message and publish it
                msg = Twist()
                msg.linear.x = v[0]
                msg.angular.z = v[5]
                self.publisher.publish(msg)

                self.new_pose_received = False
            else:
                pass
                # msg = Twist()
                # msg.linear.x = 0.0
                # msg.angular.z = -0.3
                # self.publisher.publish(msg)
                # self.get_logger().info('Waiting for a new pose...')

        except Exception as e:
            self.get_logger().error(f'Failed to compute velocity: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = VisualServoing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
