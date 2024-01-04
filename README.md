# PECORE 🤖
This repository contains the code, assignments, and projects related to the Master's course on Perception and Cognition in Robotic Exploration (PECORE) offered at UPC-ETSEIB. The course explores the fundamental concepts, algorithms, and methodologies in robot perception, cognition, and exploration. 

The repository includes implementations and solutions developed in ROS 2 and other relevant frameworks and tools, covering a range of topics such as sensor processing, mapping, localization, planning, and decision-making in robotic systems.

To complement the developed code please download the remaining dependencies where the simulations, robot configs, etc. are stored: [Download Dependencies](https://asantamarianavarro.gitlab.io/teaching/muar/pecore/src-files/pecore_2023.zip)

For more information about this exercises follow this [tutorial](https://asantamarianavarro.gitlab.io/teaching/muar/pecore/t1.intro_ros/basic/basic.html).


## Exercise 1: Simple Position Control Loop - PUBVEL Package

In this exercise, we are required to close a simple position control loop by sending velocity commands to the robot. The steps involved are as follows:

1. **Service Client Creation**:
   - Inside the `pubvel` node, create a service client to obtain the "ground-truth" pose of the robot.
   - Subscribe to the `/gazebo/get_entity_state` service.
   - In the service call, use "jackal" as the entity name and "world" as the reference frame.

2. **Desired Position**:
   - Set a hardcoded desired position for the robot in the world frame.
   - Compute the error vector between the current and desired positions.

3. **Closed loop and PID control**:
   - Set up and tune a PID to aquire the desired target pose.
   
4. **Usage**:
   - Execute the simulation:
   ```console
   ros2 launch pecore_launch tutorial1b.launch.py
   ```
   - Execute the controller:
   ```console
   ros2 launch pubvel velocity_publisher_launch.py
   ```


## Exercise 2: TF transformations usage - tf_transformations Package
In this exercise, we delve into the world of transformations and leverage ground truth data provided by the Gazebo simulation to concatenate TFs (Transformation Frames). The primary objective is to find the transformation map_H_odom by concatenating the known transformations map_H_baselink (provided by Gazebo) and odom_H_baselink (obtained through a TF listener).

The steps to achieve this are:

1. **Service Client for Ground Truth:**
    A service client is set up to communicate with the /gazebo/get_entity_state service.
    This service provides the true position and orientation of the Jackal robot in the simulated world, referenced as map_H_baselink
2. **TF Listener for odom_H_baselink:**
    A TF listener is created that waits and retrieves the transformation between the odom frame and the base_link frame (odom_H_baselink)
3. **Transformation Matrix Calculation:**
    For both transformations, we convert the transformations from TransformStamped format to a 4x4 transformation matrix.
    This is achieved through the calculate_transformation_matrix function, which takes in translation and rotation components and generates the respectivtransformation matrix
4. **Concatenate TFs to Obtain map_H_odom:**
    Using matrix multiplication and inversion, the transformation map_H_odom is obtained.
    Formula: map_H_odom=map_H_baselink×inv(odom_H_baselink)map_H_odom=map_H_baselink×inv(odom_H_baselink
5. **Broadcast the New Transformation:**
    The obtained map_H_odom matrix is converted back into a TransformStamped format and then broadcasted using a TF broadcaster.
    If performed correctly, when you set the map as the fixed frame in RVIZ, you should be able to visualize all the robot's data seamlessly
6. **Usage:**
    To execute the launch:
   ```console
   ros2 launch tf_transformations tutorial2.launch.py
   ```

   ![TF_diagram](image.png)


## Practice 1: Visual Servoing - visual_servoing_P1 Package
In this practicum you will build a node to transform the pose of a detected target into a robot command pose to drive it and approach the target using relative localization.

We developed a set of functions stored in the transformations.py file so all the solutions can share this library and clean the main codes.
To solve this problem, 3 solutions have been proposed:

1. **Navigation using the Navigation Stack using global coordinates.**
  **Usage:**
    To execute the launch:
   ```console
   ros2 launch visual_servoing_P1 ros_nav_stack_p1.launch.py
   ```

[p1_aruco-approach_NavStack_pose.webm](https://github.com/VictorEscribano/PECORE/assets/70441479/68ee7e6d-fa63-49d8-a9b7-cb5eacba77e3)

          

2. **Using custom PID controller in relative coordenates control.**
**Usage:**
    To execute the launch:
   ```console
   ros2 launch visual_servoing_P1 pid_control_p1.launch.py
   ```
   
[p1_vel_controller_pidcontrol.webm](https://github.com/VictorEscribano/PECORE/assets/70441479/8c8e58e2-8b60-4d3f-8a16-5e029bf293fa)



3. **Using position bsed visual servoing in relative coordenates control.**
**Usage:**
    To execute the launch:
   ```console
   ros2 launch visual_servoing_P1 visual_servoing.launch.py
   ```

[p1_PBVS-vel_controller_pbvs.webm](https://github.com/VictorEscribano/PECORE/assets/70441479/4d8b7501-48cd-417a-b0de-05fb9489afd7)

## Practice 2: Non-linear filters
YOu can found the info on the non_linear_filters package.

## Practice 3: Autonomous exploration:
Closest frontier approach:

https://github.com/VictorEscribano/PECORE/assets/70441479/cb1749d6-b186-4447-b885-02af46dd6e74


Own algorithm approach: LFE (Last Frontier Exploration)
he original autonomous exploration algorithm computed the
distance from the robot to frontiers in a map. However, this approach had a drawback: if the robot had already
started planning a path to a frontier and a new, closer frontier appeared during the remapping, the robot would
not adapt its path to reach the closer frontier. This led to suboptimal exploration and inefficient use of resources.
To address this issue, I made changes to the code in explore.h, explore.cpp, frontier-search.h, and frontier-
search.cpp. The key idea was to shift the focus of the cost function from the robot’s current position to the
position of the last pursued frontier goal. This allowed the robot to continuously adapt its path to the nearest
frontiers, providing a more efficient exploration strategy. So now if during a remmapping a new path needs to be
computed it will be computed taking into account the distance from the last frontier and not the distance
from the robot, making then that the following frontier chosen will be on the line of the exploration, not leading
to jumps from one point to another on the map.
The only drawback of the method is the compute method for the distance, the original explore lite package used
euclidean distance and this method does not work well on this situation as it can take a neighbour frontier that
is on the other side of the wall. To solve this the nav2 path distance should be tacked into account instead of
the euclidean distance (Not implemented).

https://github.com/VictorEscribano/PECORE/assets/70441479/830e1384-4c3f-450f-a5c7-65fc67801e9f


