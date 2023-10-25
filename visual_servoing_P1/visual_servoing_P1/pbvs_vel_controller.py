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
Descripcion: Código encargado de obtener la pose relativa entre aruco y cámara y computa el algoritmo the visual servoing basado en pose (PBVS)
"""   

# Imports from python libraries
import rclpy, transformations
import numpy as np

# Imports for the code from ros
from rclpy.node import Node	
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup			    									# Allows parallel execution of callbacks
from rclpy.executors import MultiThreadedExecutor																	# Allows to add different threads to executor so that it is not stuck on a request
from tf2_ros.transform_listener import TransformListener															# Allows to create listeners for tf transforms
from tf2_ros import TransformException																				# Allows to handle errors on the transformations done from buffer
from tf2_ros.buffer import Buffer																					# Allows to create objects to store on a buffer the tfs between frames
from geometry_msgs.msg import Twist																					# Allows to create a publisher to that message from geometry defined messages
from visual_servoing_P1.transformations import *																	# Library that allows to generate transformations between tf and homogeneous transforms among others

# Definition of the class to use with the node
class RobVelController(Node):

	def __init__(self):
		super().__init__('pbvs_vel_controller')
    	
    	# Definition of all parameters for the node
		self.declare_parameter("rob_frame","front_camera_optical") 				 									# Declaration of the desired parent frame for the transformations
		self.declare_parameter("des_frame","desired_cam_pose")														# Declaration of the desired child frame for the transformations
		self.declare_parameter("lamb_val",3.0)																		# Declaration of the lambda value to be applied to the PBVS controller
  
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
		self.tf_rob_frame="" 																						# Attribute to store which will be the robot frame on the tf buffer transform
		self.tf_des_frame=""									 													# Attribute to store which is the frame to link to the desired robot position on the tf buffer transform
		self.robHdespos=np.eye(4)																					# Attribute to store the transform between aruco and the camera 
		self.lambd_val=0
  
     	# Assign initial values from parameters
		self.tf_rob_frame=self.get_parameter("rob_frame").value 													# Get the frame to use as robot frame for tf buffer transforms
		self.tf_des_frame=self.get_parameter("des_frame").value														# Get the frame to use as desired robot frame for tf buffer transforms
		self.lambd_val=self.get_parameter("lamb_val").value															# Get the lambda value to use from launch file as desired
  									
    # Definition of function to perform when timer callback must be executed to publish a new global robot pose    
	def SetNewVelCommand(self):
    	
		vel_pose_goal = Twist()																						# Create a new instance of the Twist message anyway
		
  		# Attempt to get the transform cam_H_descampose first with data from buffer and listener
		try:
			tf_found=self.tf_buffer.lookup_transform(self.tf_rob_frame,self.tf_des_frame,rclpy.time.Time()) 		# Create the transformation first upon given data on buffer for tfs
			self.robHdespos=Rt2homo_matrix(tf_found.transform.translation,tf_found.transform.rotation)			    # Get the homogeneous transformation from the obtained tf found on the buffer
   			
      	# Handle an exception if it occurs during the transformation	
		except TransformException as ex:
			# Print on the terminal the error that occurred
			self.get_logger().info(f'Could not transform {self.tf_rob_frame} to {self.tf_des_frame}: {ex}')		
			return 0

		# Get the error pose from the relative homogeneous transform between the camera and the desired cam pose
		pos_error=np.array([self.robHdespos[0][3],self.robHdespos[1][3],self.robHdespos[2][3]])
		# Get the error angular from the relative homogeneous transform between the camera and the desired cam pose in euler angles
		ang_error=transformations.euler_from_matrix(self.robHdespos)
		# Get the final error
		total_error=np.concatenate([pos_error,ang_error])															# Concatenation of both pose and angular error
		
  		# Create the jacobian for the visual servoing approach
		J=np.eye(6)
		# Get the linear part of the jacobian
		Jp=self.robHdespos[:3,:3]
		# Get the angular part for the jacobian
		Jr=(np.trace(np.transpose(Jp))*np.identity(3)-Jp)/2
		# Concatenate both jacobians 
		J[:3,:3]=Jp;J[3:6,3:6]=Jr																					# Get the final jacobian to use on the algorithm as concatenation of both jacobians

		# Get the speeds to send to the robot
		des_vel=self.lambd_val*np.dot(np.linalg.pinv(J),np.transpose(total_error))									# Computation of the speed to be sent to the robot in all the frame
		# Set up the speed commands to send to the robot
		vel_pose_goal.linear.x=des_vel[0]																			# Set up the lineal speed as the error between poses
		vel_pose_goal.angular.z=des_vel[5]																			# Set up the angular aspeed as the angle between x and y error (yaw rate)

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

