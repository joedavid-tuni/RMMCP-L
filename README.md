# Designing, Controlling and Programming of a Lego Robot Manipulator in ROS using Raspberry Pi

## Description of the Task
The main objective is to build a manipulator that must be capable of drawing specific patterns performing rectilinear trajectories.

## Environment

This task should be performed within different elements. The physical components are shown in following Figure

Meanwhile the LEGO Mindstorms set will provide the primal material (i.e., LEGO pieces) for building the robotic arm; the combination of i) the Raspberry Pi and ii) the BrickPi boards will construct a controller that will be used for hosting the required programming code for the robot to accomplish the task. 

## Task Requirements
The challenge of the assignment is to construct a robotic arm of at least 3 DOM (no Cartesian structure) that is capable of draw a selected mobile variant shown in following Figure 3. Each mobile variant is composed by three main parts: screen, keyboard and frame. Then, the challenge consists on drawing each part in a same piece of paper, simulating the assembly process of mobile components.

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
