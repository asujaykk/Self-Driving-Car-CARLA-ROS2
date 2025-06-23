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




### 1. 🌍 Start **CARLA server** and source  CARLA ROS2 bridge. 
1.  **Run the CARLA server.**
    Open a new terminal (terminal 1) and run the following command to start the CARLA server:

    ```bash
    bash $HOME/CARLA_PROJECT/CARLA_0.9.15/CarlaUE4.sh -prefernvidia
    ```

2.  **Source ROS2 HUMBLE.**
    Open a new terminal (terminal 2) and run the following command to source ROS2:

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    ```

3.  **🔗 Source the ROS2 CARLA Bridge.**
    In terminal 2, run the following command:

    ```bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    ```

### 2. 🚗 Spawn a vehicle
1.  Use a **ROS2 package to spawn a vehicle**.
    In terminal 2, run the following command:

    ```bash
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
    ```

### 3. 🧭 Use a route planner to find the best route from the vehicle's location to the destination point.
1.  For route planning, we will be using the **waypoint publisher**, which is part of the ROS2 package.
    To initiate the waypoint publisher, run the following command in a new terminal (terminal 3):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
    ```

### 4. ⚙️ Use a controller to control the vehicle to follow the route.
1.  We will be using the **local planner package** in the CARLA ROS2 bridge for controlling the vehicle or to follow the published waypoints (route) in CARLA.
    To initiate the local planner, please run the following command from a new terminal (terminal 4) as follows:

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    ros2 run carla_ad_agent local_planner
    ```


### 5. 🎯 Use Navigation HMI to select the destination location on the map.
1.  **Source the navigation package.**
    Open a new terminal (terminal 5) and run the following command:

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
    ```

2.  **Run the navigation node** with the following command:

    ```bash
    ros2 run navigation navigation_hmi
    ```

3.  **Launch RViz 2** in a new terminal (terminal 6):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash    
    rviz2
    ```

4.  Select the `carla_road_network` topic to **visualize the road network** in RViz 2.
5.  Select the `carla/markers` topic to **visualize the vehicle** in RViz 2.
6.  Select the `carla/waypoints` topic to **Visualize the published route** in Rviz 2.
7.  Now choose the **"Goal Pose" option** to select the destination point in the road network.
8.  Once you select the goal pose, you can **see the new route created by the waypoint publisher** on the map.
9.  And at this moment, the **vehicle will start driving towards this location**.

## Video tutorial
https://youtu.be/jFORY41LDHo?si=h1kVquHgi4Zu2OJq
