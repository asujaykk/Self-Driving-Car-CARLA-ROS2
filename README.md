# Self-Driving-Car-CARLA-ROS2
A project to develope a self driving car with carla simulator and ROS2 carla bridge

Here we are aiming to build a software stack step by step to drive a car in CARLA simulator autonomously with ROS2 humble

# Software Requirements
1. Ubuntu 22.04
2. ROS2 Humble
3. CARLA 9.15.0
4. CARLA ROS2 HUMBLE bridge
5. python 3.12.0

# What is a self driving car?

A self-driving car is a vehicle equipped with sensors, software, and control systems that enable it to navigate and drive itself without human input.
It perceives its environment (roads, traffic, pedestrians, obstacles), makes decisions, and controls motion (steering, acceleration, braking) to reach a destination safely and efficiently.

Self-driving cars use technologies like:
1. Sensors: cameras, lidar, radar, ultrasonic sensors.
2. Software: perception algorithms, planning, control, and decision-making systems..
3. AI/ML: for object detection, path planning, behavior prediction
4. Connectivity: GPS, HD maps, sometimes V2X (vehicle-to-everything) communication.


# What we  need to build a self drivingf car?
1. A vehicle
2. Required sensors attached to the vehicle.
3. A control system/master processing unit.
4. Actuators to control the vehicle based on the constrol signals from the controller.
5. A world to test the vehicle with different road conditions and scenarios.

As we know creating a test vehicle with above requirements is expensive also testing the vehicle with different scenarios is not easy job.
Instead the easy way is to use a simulator where we can create vehicle with complex sensors within in a virtual world.

In this project are going to use CARLA SIMULATOR , it is really heavy simulator (consider a performance laptop/workstation).
And we can read the sensor data from this vehicle with python API/ ROS bridge.
Also can control the vehicle with API/ ROS bridge

# Prerequisite
1. A PC with ubuntu 22.04 OS installed (with dedicated graphics)
2. ROS2 Humble installed
3. CARLA 9.15.0 simulator installed
4. CARLA ROS2 bridge installed

# PHASE-1
1. In this phase, the basic objective of a self driving car will be implemented
2. The basic objective is to drive a car from a starting point to a destination point.
3. Here we are not bothering about collirtions, traffic rules or complex menuvers

## how to achieve this
1. Get the complete road map of CARLA WORLD,, which is equivalent to google map.
2. Spawn a vehicle in simulator.
3. Create An HMI to select destination location in the map.
4. Use route planner to find the best route from vehicle location to destination point.
5. Also run the controller to control the vehicle to follow the route

### Step by step operation to do it with CARLA , ROS2 , CARLA ROS2 bridge
#### 1.Get the complete road map of CARLA WORLD,, which is equivalent to google map.
1. Run carla server.
open a new terminal (terminal 1) and run following command to start carla server.

       bash <CARLA_EXECUTABLE_DIRECTORY>/CarlaUE4.sh  -prefernvidia
   
3. Source ROS2 HUMBLE
open a new terminal(terminal 2) and run following command to source ROS2

       source /opt/ros/humble/setup.bash
4. Source ROS2 CARLA Bridge
In terminal 2 , run following command
 
        source /home/akhil_msi/Workspace/ros-bridge/install/setup.bash
        export CARLA_ROOT=/home/akhil_msi/WORKING_FOLDER/CARLA_0.9.15
        export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
   
6. Spawn example ego vehicle
7. 




