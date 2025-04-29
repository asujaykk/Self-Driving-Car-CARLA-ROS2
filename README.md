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
1. A PC with ubuntu 22.04 OS installed (with dedicated graphics).
2. ROS2 Humble installed.
   follow official page of ROS2 for installation https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
4. CARLA 9.15.0 simulator installed
   download the binary installation file from CARLA offcial github page, extract it in to your PC
6. CARLA ROS2 bridge installed
   follow following link for CARLA ROS2 bridge installation https://github.com/ttgamage/carla-ros-bridge
7. Install this repository to your WORKING DIRECTORY

          git clone git@github.com:asujaykk/Self-Driving-Car-CARLA-ROS2.git
8. build the package and source it

          cd Self-Driving-Car-CARLA-ROS2
          colcon build
          source install/setup.bash



# PHASE-1  : A SIMPLE ROUTE FOLLOWING VEHICLE (WAYPOINT FOLLOWER)
1. In this phase, the basic objective of a self driving car will be implemented
2. The basic objective is to drive a car from a starting point to a destination point.
3. Here we are not bothering about collirtions, traffic rules or complex menuvers

## how to achieve this
1. Get the complete road map of CARLA WORLD,, which is equivalent to google map.
2. Spawn a vehicle.
3. Use route planner to find the best route from vehicle location to destination point.
4. Use a controller to control the vehicle to follow the route
5. Use Navigatiobn HMI to select destination location in the map.

### Step by step operation to do it with CARLA , ROS2 , CARLA ROS2 bridge
#### 1.Get the complete road map of CARLA WORLD,, which is equivalent to google map.
1. Run carla server.
open a new terminal (terminal 1) and run following command to start carla server.

       bash <CARLA_EXECUTABLE_DIRECTORY>/CarlaUE4.sh  -prefernvidia
   
2. Source ROS2 HUMBLE
open a new terminal(terminal 2) and run following command to source ROS2

       source /opt/ros/humble/setup.bash
3. Source ROS2 CARLA Bridge
In terminal 2 , run following command
 
        source /home/akhil_msi/Workspace/ros-bridge/install/setup.bash
        export CARLA_ROOT=/home/akhil_msi/WORKING_FOLDER/CARLA_0.9.15
        export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
   
#### 2. Spawn a vehicle
1. use ROS2 package to spawn a vehicle. 
In terminal 2 , run following command

       ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

#### 3. Use route planner to find the best route from vehicle location to destination point.
1. For route planning we will be using waypoint publisher which is part of ROS2 package.
   to initiate waypoint publisher run following command in a new terminal (terminal 3)

        ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py  

#### 4. Use a controller to control the vehicle to follow the route
1. We will be using the local planner package in carla ros2 bridge For controlling the vehicle or to follow the published waypoints (route) in carla.
   To initate the local planner please run the following command from a new terminal (terminal 4) as follows,

          ros2 run carla_ad_agent local_planner


#### 5. Use Navigatiobn HMI to select destination location in the map. 
1. Source navigation package
   Open a new terminal (terminal 5) and run following command
       
         source install/setup.bash
2. Run navigation node with following command

          ros2 run navigation navigation_hmi
3. Open a new terminal (terminal 6) and run following command to open rviz2

          rviz2
5. select the carla_road_network topic to visualize the road network in rviz2
6. select carla\here\marker topic to visualize the vehicle in rviz2
7. Now chose goal pose option to select the destination point in the road network.
8. Once you select the goal pose, you can see the new route created by waypoint publisher in the map
9. And at this moment the vehicle will start driving towwards this location.



           
