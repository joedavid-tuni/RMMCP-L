# Designing, Controlling and Programming of a Lego Robot Manipulator in ROS using Raspberry Pi

## Description of the Task
The main objective is to build a manipulator that must be capable of drawing specific patterns performing rectilinear trajectories.

## Environment

This task should be performed within different elements. The physical components are shown in following Figure

<a href="https://drive.google.com/uc?export=view&id=1u_hwLCdNbtflsW_Zq8Kdbf-KI7soF7m_"><img src="https://drive.google.com/uc?export=view&id=1u_hwLCdNbtflsW_Zq8Kdbf-KI7soF7m_" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


Meanwhile the LEGO Mindstorms set will provide the primal material (i.e., LEGO pieces) for building the robotic arm; the combination of i) the Raspberry Pi and ii) the BrickPi boards will construct a controller that will be used for hosting the required programming code for the robot to accomplish the task. 

## Task Requirements
The challenge of the task is to construct a robotic arm of at least 3 DOM (no Cartesian structure) that is capable of draw a selected mobile variant shown in following Figure 3. Each mobile variant is composed by three main parts: screen, keyboard and frame. Then, the challenge consists on drawing each part in a same piece of paper, simulating the assembly process of mobile components.

<a href="https://drive.google.com/uc?export=view&id=1lmnCwlOxJhsw5E6xZFYkhsHglUeEcpzn"><img src="https://drive.google.com/uc?export=view&id=1lmnCwlOxJhsw5E6xZFYkhsHglUeEcpzn" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


A program must be written in order to define and create the required logics of robot operations with the above mentioned equipments and the objectives must be achieved  

----------------------------------------------------------------------------------------------------------------------------------------


# Solution

## Programming Environment
* Language: Python, C++, Matlab
* Framework: ROS, Ubuntu
* IDE: PyCharm, QT, Matlab

It was decided to use the Open Source ROS framework to achieve the objectives of the task as there are several useful libraries available
The ROS environment provides framework to communicate different programs in different languages on the same or even different devices on the same network.

Two nodes written in  Python and C++ languages are coded in their respective single text file. 
The node coded in C++, **Planner.cpp** is meant to provide values for the robot joints to be reached given a trajectory to be followed. In addition, it subscribes for strings to be received and parsed with the desired trajectory data and it performs required forward and inverse kinematics, using the kinematic model. Finally, it publishes a set of vectors of angles within the defined trajectory.

The node coded in Python, **Task.py** is meant to contain  the tasks to be performed depending on the numbers received in the number topic, it also includes a function to receive and perform the trajectory that is sent by the planner upon request.

**MATLAB** is used to publish messages using the official Robotics Toolbox  indicating what is going to be drawn. For achieving this, both task and planner nodes must be first executed, then after execution of the provided and correctly modified MATLAB script, a command to the python node will be sent to draw the requested number.

To summarize, there are four main elements to provide a solution for the task:
* Manipulator of 3 DOM built by the provided LEGO set. The construction of the robotic arm will require the employment of LEGO motors, pieces and probably sensors.
* Node for controlling the manipulator in Python.
* Node for the kinematic model of your robot in C++.
* MATLAB with Robotics Toolbox to send numbers to the Raspberry Pi. Note that TUT computers may not be able to correctly send the commands.

## Robot Design

A scara robot is made with the Lego Kit as shown in the figure below.

### Side View
<a href="https://drive.google.com/uc?export=view&id=1kP_I4r9hlDeZcc1mWf8lF7xEx2A867WP"><img src="https://drive.google.com/uc?export=view&id=1kP_I4r9hlDeZcc1mWf8lF7xEx2A867WP" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

### Gear MEchanism and Prismatic Joint
<a href="https://drive.google.com/uc?export=view&id=1TLTNq6n6aNiQ4lhlGkJ1253EKDy1AjrO"><img src="https://drive.google.com/uc?export=view&id=1TLTNq6n6aNiQ4lhlGkJ1253EKDy1AjrO" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

There are two gear mechanisms, one spur gear mounted on the base link and another worm gear mounted on the end of the second link. The spur gear ratio was designed to have a lower speed but higher torque as the base revolute joint has a major portion of the robot arm attached to it, hence larger is the weight and inertia. Two gears having 24 and 56 teeth are mounted on parallel shafts to obtain the first revolute joint (gear ratio 0.428). The second revolute joint has no gear mechanism, or has a gear ratio of 1

The prismatic joint consists of a worm gear arrangement. A 12 teeth gear is mounted on the motor shaft, while a ‘ganged gears’ on a different but same shaft, one with 12 and the other with 48 teeth is driven by the gear on the motor shaft and drives the ‘worm screw’ that effects the linear movement of the prismatic joint. The prismatic joint has a vertical extension of about a centimeter as no more was necessary to lift and drop the pen in the context of this task.

The SCARA Lego robot has is similar to a conventional SCARA robot except that it doesn’t have the revolute joint on the last link with a prismatic joint*
Size
The Robot has a base footprint of 19x12cm
Weight
800grams
Reachable Workspace
The Lego Robot has a reachable workspace of a circle with diameter equal to the sum of the first two links.**
Gear Mechanisms

The base link of the SCARA Lego Robot is connected to the first link via a gear mechanism. There are no gear mechanisms between the first and second links while a worm gear arrangement exists to convert the rotational motor motion into linear prismatic movement 

## Kinematic Model

<a href="https://drive.google.com/uc?export=view&id=1dbwDAa1VZSNxCeoZUCSWUWnSQ_5wYYin"><img src="https://drive.google.com/uc?export=view&id=1dbwDAa1VZSNxCeoZUCSWUWnSQ_5wYYin" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

## Gear Ratio Selection

<a href="https://drive.google.com/uc?export=view&id=1tEkHG4hcjS8uDVZrTdD4cfmF7ClPaiPE"><img src="https://drive.google.com/uc?export=view&id=1tEkHG4hcjS8uDVZrTdD4cfmF7ClPaiPE" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>



Here we look at the arbitrary example of an 8 teeth gear (driving )and a 24 teeth gear (driven). In terms of torque, since the forces between the teeth of the two gears are equal in magnitude but act opposite in directions. (This is a direct consequence of Newton’s 3rd Law of Motion.) Therefore, the torque exerted on the right axle is three times the torque exerted on the left axle (since the radii of thee gears differ by a factor of three). Thus this gear system as acts as a “torque converter”, increasing the torque at the expense of decreasing the rate at which the axle turns. Since torque = r x F, the torque about axle #2 is three times greater than the torque about axle #1; the gear train acts as a “torque amplifer”.

## Drawing Resolution

<a href="https://drive.google.com/uc?export=view&id=1GCZ1_WHFfkvhH-CnKBB-LY9BqwzlQjkf"><img src="https://drive.google.com/uc?export=view&id=1GCZ1_WHFfkvhH-CnKBB-LY9BqwzlQjkf" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>



The models were made in CAD to predict the feasible resolution of the mobile phones based on the reachable workspace of the SCARA Robot. The biggest possible resolution was studied as the SCARA Robot has lesser accuracy at smaller resolutions. Dimensions were taken to transfer them to the python code.

## Source Code

### Trajectory Definition

The assignment trajectory was seen as a problem of drawing two basic primitives, lines and rectangles. 
These functions accepts the x and y coordinates of the starting position (bottom left position in case of rectangles), the offset values( the height and width of the rectangle or length of the line) and the number of Points (nPoints).Thus by using these parameters any rectangle of arbitrary height and width in an arbitrary position or a vertical/horizontal line of an arbitrary length in an arbitrary position can be drawn

Pid controller parameters were adjusted to allow a smooth trajectory based on trial and error. The toleration error threshold for joint 1was increased to 7 degrees because first motor doesn’t move for an angular error below 6 degrees due to static friction

## Drawn Specimen

<a href="https://drive.google.com/uc?export=view&id=1Oz_zx2fGlCSyxCNRqSGmXue0u1Ufvcdm"><img src="https://drive.google.com/uc?export=view&id=1Oz_zx2fGlCSyxCNRqSGmXue0u1Ufvcdm" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

