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
Descripcion: Codigo encargado de enviar comandos de velocidad basados en el controlador PI.
"""  
# Imports from python libraries
import rclpy
import numpy as np

# Imports for the code from ros
from rclpy.node import Node	
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup			    									# Allows parallel execution of callbacks
from rclpy.executors import MultiThreadedExecutor																	# Allows to add different threads to executor so that it is not stuck on a request
from tf2_ros.transform_listener import TransformListener															# Allows to create listeners for tf transforms
from tf2_ros import TransformException																				# Allows to handle errors on the transformations done from buffer
from tf2_ros.buffer import Buffer																					# Allows to create objects to store on a buffer the tfs between frames
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped  												# Allows to create a publisher to that message from geometry defined messages
from visual_servoing_P1.transformations  import *																	# Library that allows to generate transformations between tf and homogeneous transforms among others

# Definition of the class to use with the node
class RobVelController(Node):

	def __init__(self):
		super().__init__('jackal_vel_controller')
    	
    	# Definition of all parameters for the node
		self.declare_parameter("main_frame","map")																	# Declaration of the desired parent frame for the transformations
		self.declare_parameter("rob_frame","base_link")																# Declaration of the desired parent frame for the transformations
		self.declare_parameter("des_frame","des_rob_pose")															# Declaration of the desired child frame for the transformations
		self.declare_parameter("aruco_frame","aruco_marker_frame")													# Declaration of the desired child frame for the transformations
		self.declare_parameter("kp_pos",2.0)																		# Declaration of the desired Kp for the robot error in position controller
		self.declare_parameter("ki_pos",0.1)																		# Declaration of the desired Ki for the robot error in position controller
		self.declare_parameter("kp_angle",5.0)																		# Declaration of the desired Kp for the robot error in orientation controller
		self.declare_parameter("ki_angle",0.1)																		# Declaration of the desired Kp for the robot error in orientation controller
	
     	# Definition of all necessary callback groups
		self.pub_group=MutuallyExclusiveCallbackGroup()																# Callback groups for the publisher messages
	
		# Definition of all publishing and subscribing methods
		self.pub_pose_pub = self.create_publisher(Twist,'/jackal_velocity_controller/cmd_vel_unstamped', 100)		# Creation of the publisher for the node with the speed for the jackal
  
		# Definition of all listener methods for the node
		self.tf_buffer = Buffer()																					# Create the buffer to store all transforms between frames
		self.tf_listener=TransformListener(self.tf_buffer, self)													# Create the listener for the transformations from tf done between odom and link base														
    	
    	# Definition of timer methods
		timer_period= 0.5 																							# Default timer seconds to publish a new command for pose or speed
		self.timer= self.create_timer(timer_period, self.SetNewVelCommand, callback_group=self.pub_group)			# Definition of timer with the function to be called on callbacks for publishing new commands
     
    	# Definition of attributes for the class
		self.tf_world_frame=""																						# Attribute to store which will be the reference frame on the tf buffer transform to be done
		self.tf_rob_frame="" 																						# Attribute to store which will be the robot frame on the tf buffer transform
		self.tf_des_frame=""									 													# Attribute to store which is the frame to link to the desired robot position on the tf buffer transform
		self.robHdespos=np.eye(4)																					# Attribute to store the transform between aruco and the camera 
		self.pos_error=0																							# Attribute to store the accumulated error between executions of linear velocity commands
		self.angle_error=0																							# Attribute to store the accumulated error between executions of angular velocity commands
		self.kp_pos=0																								# Attribute to store the kp value that will be used on position controller
		self.kp_angle=0																								# Attribute to store the kp value that will be used on orientation controller
		self.ki_pos=0																								# Attribute to store the ki value that will be used on position controller
		self.ki_angle=0																								# Attribute to store the ki value that will be used on orientation controller
    	
     	# Assign initial values from parameters
		self.tf_world_frame=self.get_parameter("main_frame").value													# Get the frame to use as world reference for tf buffer transforms
		self.tf_rob_frame=self.get_parameter("rob_frame").value 													# Get the frame to use as robot frame for tf buffer transforms
		self.tf_des_frame=self.get_parameter("des_frame").value														# Get the frame to use as desired robot frame for tf buffer transforms
		self.tf_aruco_frame=self.get_parameter("aruco_frame").value													# Get the frame to use as desired robot frame for tf buffer transforms
		self.kp_pos=self.get_parameter("kp_pos").value																# Get the value for the kp introduced for position controller by user on the attribute
		self.kp_angle=self.get_parameter("kp_angle").value															# Get the value for the kp introduced for orientation controller by user on the attribute
		self.ki_pos=self.get_parameter("ki_pos").value																# Get the value for the ki introduced for position controller by user on the attribute
		self.ki_angle=self.get_parameter("ki_angle").value															# Get the value for the ki introduced for orientation controller by user on the attribute

		# Desired camera pose w.r.t. the target (ArUco marker)
		self.aruco_H_desPos = TransformStamped()
		self.aruco_H_desPos.transform.translation.x = 0.0
		self.aruco_H_desPos.transform.translation.y = 0.0
		self.aruco_H_desPos.transform.translation.z = 0.1
		self.aruco_H_desPos.transform.rotation.x = -0.707
		self.aruco_H_desPos.transform.rotation.y = 0.707
		self.aruco_H_desPos.transform.rotation.z = 0.0
		self.aruco_H_desPos.transform.rotation.w = 0.707
		self.aruco_H_desPos.header.frame_id = self.tf_aruco_frame
		self.aruco_H_desPos.child_frame_id = self.tf_des_frame
		self.aruco_H_desPos.header.stamp = self.get_clock().now().to_msg()			

    # Definition of function to perform when timer callback must be executed to publish a new global robot pose    
	def SetNewVelCommand(self):
    	
		vel_pose_goal = Twist()																						# Create a new instance of the Twist message anyway
		
		# Attempt to get the transform map_H_actpose first with data from buffer and listener
		try:
			map_H_robot = self.tf_buffer.lookup_transform(self.tf_world_frame,self.tf_rob_frame,rclpy.time.Time()) 	# Create the transformation first upon given data on buffer for tfs
			mapHactrob=Rt2homo_matrix(map_H_robot.transform.translation,map_H_robot.transform.rotation)				# Get the homogeneous transformation from the obtained tf found on the buffer
    
        # Handle an exception if it occurs during the transformation
		except TransformException as ex:
			# Print on the terminal the error that occurred
			self.get_logger().info(f'Could not transform {self.tf_world_frame} to {self.tf_rob_frame}: {ex}')
			return 0																								# Escape from the callback if an error occurred

		# Attempt next to get the transform map_H_despose with data from buffer and listener
		try:
			mapHaruco_tf=self.tf_buffer.lookup_transform(self.tf_world_frame, self.tf_aruco_frame, rclpy.time.Time()) # Create the transformation first upon given data on buffer for tfs
			mapHaruco=Rt2homo_matrix(mapHaruco_tf.transform.translation,mapHaruco_tf.transform.rotation)			  # Get the homogeneous transformation from the obtained tf found on the buffer
			arucoHdesired_tf = Rt2homo_matrix(self.aruco_H_desPos.transform.translation, self.aruco_H_desPos.transform.rotation)
			mapHdespos = np.dot(mapHaruco, arucoHdesired_tf)

      	# Handle an exception if it occurs during the transformation	
		except TransformException as ex:
			# Print on the terminal the error that occurred
			self.get_logger().info(f'Could not transform {self.tf_rob_frame} to {self.tf_des_frame}: {ex}')		
			return 0																								# Escape from the callback if an error occurred
			
		# Next, get the homogeneous transformation that goes from actual robot pose to desired robot pose
		self.robHdespos=np.dot(np.linalg.inv(mapHactrob),mapHdespos) 												# Obtain directly the transformation for the transform between actual robot pose and desired robot pose

		# Get from the homogeneous transformation the distance between the actual robot position and the desired pose
		pos_error=np.sqrt(self.robHdespos[0][3]**2+self.robHdespos[1][3]**2)										# Get the error from relative position (x,y) between robot and desired pose
		angle_error=np.arctan2(self.robHdespos[1][3],self.robHdespos[0][3])											# Get the error wrt to the angle to be applied depending on the error in x,y coordinates
		self.pos_error+=pos_error																					# Sum up the error with the actual one
		self.angle_error+=angle_error

		# Set up the speed commands to send to the robot
		vel_pose_goal.linear.x=self.kp_pos*pos_error+self.ki_pos*pos_error											# Set up the lineal speed as the error between poses
		vel_pose_goal.angular.z=self.kp_angle*angle_error+self.ki_angle*self.angle_error							# Set up the angula aspeed as the angle between x and y error (yaw rate)

   		# Send the message broadcasted
		self.pub_pose_pub.publish(vel_pose_goal)																	# Publish the data of the velocity command	

# Main function that will be called from the script
def main(args=None):

	# Set up libraries
	rclpy.init(args=args)														# Start a new context and global executor
    	
    # Set up nodes
	jackal_controller = RobVelController()										# Create a node object from the class designed	
    	
    # Set up the executor
	executor_obj=MultiThreadedExecutor(num_threads=2)							# Create an executor instance multithreaded with 2 threads
	executor_obj.add_node(jackal_controller)									# Add to the executor on one thread the node to execute all callbacks
    	
    # Run indefinitely the executor unless the node is destroyed
	try:
		executor_obj.spin()														# Main loop for the code
	# Raise a message on the terminal in case the user pressed CTRL+C to stop the loop	
	except KeyboardInterrupt:
			jackal_controller.get_logger().info("User wanted to abort code! All routines will be finised")	# Show message in case of interruption from user
	finally:
		# Destroy the node explicitly
    	# (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
		jackal_controller.destroy_node()										# Destroy the node already created					
		executor_obj.shutdown(timeout_sec=0.0)									# Stops the generated executor				
	
	#rclpy.shutdown()															# Shutdowns the generated context and global executor (not necessary)

# Function that needs to be executed in case the script name is called
if __name__ == '__main__':
	main()

