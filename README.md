# PECORE 🤖
This repository contains the code, assignments, and projects related to the Master's course on Perception and Cognition in Robotic Exploration (Percepció i Cognició en l'Exploració Robòtica - PECORE) offered at UPC-ETSEIB. The course explores the fundamental concepts, algorithms, and methodologies in robot perception, cognition, and exploration. The repository aims to serve as a resource for students, educators, and practitioners in the field of robotics and autonomous systems. It includes implementations and solutions developed in ROS 2 and other relevant frameworks and tools, covering a range of topics such as sensor processing, mapping, localization, planning, and decision-making in robotic systems.

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
