# 🚗 Self-Driving-Car-CARLA-ROS2 🚀
A project to **develop a self-driving car** using the **CARLA simulator** and **ROS2 CARLA bridge**.

Here, we are aiming to build a software stack step by step to **drive a car in the CARLA simulator autonomously** with **ROS2 Humble**.

## ⚙️ Software Requirements
1.  **Ubuntu 22.04**
2.  **ROS2 Humble**
3.  **CARLA 0.9.15**
4.  **CARLA ROS2 HUMBLE bridge**
5.  **Python 3.10.12**

## 🤔 What is a self-driving car?

A **self-driving car** is a vehicle equipped with **sensors, software, and control systems** that enable it to **navigate and drive itself without human input**.
It **perceives its environment** (roads, traffic, pedestrians, obstacles), **makes decisions**, and **controls motion** (steering, acceleration, braking) to **reach a destination safely and efficiently**.

Self-driving cars use technologies like:
1.  **Sensors:** cameras, lidar, radar, ultrasonic sensors.
2.  **Software:** perception algorithms, planning, control, and decision-making systems.
3.  **AI/ML:** for object detection, path planning, behavior prediction.
4.  **Connectivity:** GPS, HD maps, sometimes V2X (vehicle-to-everything) communication.


## 🛠️ What do we need to build a self-driving car?
1.  A vehicle
2.  Required sensors attached to the vehicle.
3.  A control system/master processing unit.
4.  Actuators to control the vehicle based on the control signals from the controller.
5.  A world to test the vehicle with different road conditions and scenarios.

As we know, creating a test vehicle with the above requirements is expensive, and testing the vehicle with different scenarios is not an easy job.
Instead, the **easy way is to use a simulator** where we can create a vehicle with complex sensors within a virtual world.

In this project, we are going to use the **CARLA SIMULATOR** – a really heavy simulator (consider a performance laptop/workstation).
And we can **read the sensor data** from this vehicle with the **Python API/ROS bridge**.
Also, we can **control the vehicle** with the **API/ROS bridge**.

## ⚠️ Prerequisite
1.  A PC with **Ubuntu 22.04 OS** installed (with dedicated graphics).
2.  **ROS2 Humble** installed.

    Follow the official page of ROS2 for installation: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
4.  **CARLA 0.9.15.0 simulator** installed.

    create A folder named 'CARLA_PROJECT' for all data
    
        mkdir CARLA_PROJECT
    create workspace folder for carla ros2 bridge

        mkdir CARLA_PROJECT/ros2_bridge_ws
    create workspace folder for our project software

        mkdir CARLA_PROJECT/carla_ros2_ws
    
    Download the binary installation file from the CARLA official GitHub page and extract it to your CARLA_PROJECT folder.

        cd CARLA_PROJECT
        wget wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz
        mkdir  CARLA_0.9.15 
        tar -xzvf CARLA_0.9.15.tar.gz -C CARLA_0.9.15
   
    
6.  **CARLA ROS2 bridge** installed.

    Follow the link for CARLA ROS2 bridge installation: [https://github.com/ttgamage/carla-ros-bridge](https://github.com/ttgamage/carla-ros-bridge)
    Open a new terminal and clone the repository

           mkdir -p ~/CARLA_PROJECT/ros2_bridge_ws/ros-bridge && cd ~/CARLA_PROJECT/ros2_bridge_ws/ros-bridge
           git clone --recurse-submodules https://github.com/ttgamage/carla-ros-bridge.git
           mv carla-ros-bridge src
    Set up ROS environment and install dependencies.

           conda deactivate
           source /opt/ros/humble/setup.bash
           rosdep update
           rosdep install --from-paths src --ignore-src -r
    Build the ROS bridge workspace using colcon.
  
           colcon build --symlink-install
8.  Install this repository to your **WORKING DIRECTORY**:
   
    Open a new terminal and navigate to 'CARLA_PROJECT/carla_ros2_ws' folder
    ```bash
    conda deactivate
    cd CARLA_PROJECT/carla_ros2_ws
    mkdir src
    cd src
    git clone git@github.com:asujaykk/Self-Driving-Car-CARLA-ROS2.git
    
    ```
    Build the package and source it:

    ```bash
    cd ..
    colcon build
    source install/setup.bash
    ```



## 🚦 PHASE-1: A SIMPLE ROUTE FOLLOWING VEHICLE (WAYPOINT FOLLOWER)
1.  In this phase, the **basic objective of a self-driving car will be implemented**.
2.  The basic objective is to **drive a car from a starting point to a destination point**.
3.  Here, we are **not bothering about collisions, traffic rules, or complex maneuvers**.

### 🕹️ Step-by-step operation to do it with CARLA, ROS2, CARLA ROS2 bridge
1.  Start **CARLA server** and source  CARLA ROS2 bridge. 
2.  **Spawn a vehicle**.
3.  Use a **route planner** to find the best route from the vehicle's location to the destination point.
4.  Use a **controller** to control the vehicle to follow the route.
5.  Use **Navigation HMI** to select the destination location on the map.

Please follow procedure mentioned in [PHASE-1_readme.MD](https://github.com/asujaykk/Self-Driving-Car-CARLA-ROS2/blob/main/PHASE-1_readme.MD)

### Video tutorial
https://youtu.be/jFORY41LDHo?si=h1kVquHgi4Zu2OJq


## 🚦 PHASE-2: Implementing Collision avoidance.
1.  The waypoint following vehicle in phase-1 do not avoid collisions and traffic rules.
2.  In Phase-2 we will add **collision avoidance control to the vehicle**.

### 🕹️ Step-by-step operation to do it with CARLA, ROS2, CARLA ROS2 bridge
1.  Start **CARLA server** and source  CARLA ROS2 bridge. 
2.  **Spawn a vehicle**.
3.  Use a **route planner** to find the best route from the vehicle's location to the destination point.
4.  Use a **controller** to control the vehicle to follow the route.
5.  Use **Navigation HMI** to select the destination location on the map.
6.  use **collision monitor** module to monitor frontal collision and control the vehicle to avoid it.

Please follow procedure mentioned in PHASE-2_readme.MD

### Video tutorial

