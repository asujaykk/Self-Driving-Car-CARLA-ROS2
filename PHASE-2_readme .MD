 ## 🚦 PHASE-2: Add collision avoidance feature to the EGO VEHICLE.
1.  In this phase, we will solve one real world problem of a self driving car: **collision avoidance**.
2.  The ego vehicle in PHASE-1 is simply following the route. Now we will add another ros2 node for,
    1. Predicting possible frontal colliion
    2. Intitating required breaking/EMERGENCY BREAKING to to avoid collision

### 🕹️ Step-by-step operation to do it with CARLA, ROS2, CARLA ROS2 bridge
1.  Start **CARLA server** and source  CARLA ROS2 bridge. 
2.  **Spawn a vehicle** with custom vehicle definition file.
3.  Use a **route planner** to find the best route from the vehicle's location to the destination point.
4.  Use a **controller** to control the vehicle to follow the route.
5.  Use **Navigation HMI** to select the destination location on the map.

### Clone this repository to your 'CARLA_PROJECT/carla_ros2_ws/src' folder*:
Open a new terminal and navigate to 'CARLA_PROJECT/carla_ros2_ws/src' folder, then remove **Self-Driving-Car-CARLA-ROS2** folder and freshly clone it
    
   ```bash
    conda deactivate
    cd $HOME/CARLA_PROJECT/carla_ros2_ws/src
    git clone git@github.com:asujaykk/Self-Driving-Car-CARLA-ROS2.git
   
    ```

Open a new terminal and Build the package and source it:

    ```bash
    conda deactivate
    cd $HOME/CARLA_PROJECT/carla_ros2_ws
    colcon build
    source install/setup.bash
    ```

### 1. 🌍 Start CARLA server.
1.  **Run the CARLA server.**
    Open a new terminal (terminal 1) and run the following command to start the CARLA server:

    ```bash
    bash <CARLA_EXECUTABLE_DIRECTORY>/CarlaUE4.sh -prefernvidia
    ```


### 2. 🚗 Spawn a vehicle with custom vehicle definition file.
1.  Use a **ROS2 package to spawn a vehicle**. with custom vehicle definition file **car_definition_file.json**
    In terminal 2, run the following command in a new terminal (terminal 2):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py objects_definition_file:=$HOME/CARLA_PROJECT/carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2/car_definition_file.json
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
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
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
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
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

2.  **Run the navigation node** with the following command (disable publishing the speed command):

    ```bash
    ros2 run navigation navigation_hmi  --ros-args -p enable_spd_cmd:=false
    ```

3.  **Launch RViz 2** in a new terminal (terminal 6):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    rviz2
    ```

4.  Select the `carla_road_network` topic to **visualize the road network** in RViz 2.
5.  Select the `carla/here/marker` topic to **visualize the vehicle** in RViz 2.
6.  Now choose the **"Goal Pose" option** to select the destination point in the road network.
7.  Once you select the goal pose, you can **see the new route created by the waypoint publisher** on the map.
