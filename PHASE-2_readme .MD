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
5.  Use **collsion avoidance system** to monitor and avoid frontal collision. 
6.  Use **Navigation HMI** to select the destination location on the map.
7.  Use **CARLA PYTHONAPI example** scripts to launch another vehicle in the simulator and test the behavior of ego vehicle.   

### Clone this repository to your 'CARLA_PROJECT/carla_ros2_ws/src' folder*:
Open a new terminal and navigate to `CARLA_PROJECT/carla_ros2_ws/src` folder, then remove **Self-Driving-Car-CARLA-ROS2** folder and freshly clone it:
    

    conda deactivate
    source /opt/ros/humble/setup.bash
    cd $HOME/CARLA_PROJECT/carla_ros2_ws/src
    rm -rf Self-Driving-Car-CARLA-ROS2
    git clone https://github.com/asujaykk/Self-Driving-Car-CARLA-ROS2.git


 Build the package and source it:

    cd $HOME/CARLA_PROJECT/carla_ros2_ws
    rm -rf build
    rm -rf install
    rm -rf log
    colcon build
    source install/setup.bash


### 1. 🌍 Start CARLA server.
1.  **Run the CARLA server.**
    Open a new terminal (terminal 1) and run the following command to start the CARLA server:

    ```bash
    bash $HOME/CARLA_PROJECT/CARLA_0.9.15/CarlaUE4.sh -prefernvidia
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

### 5. ⚙️ Use **collsion avoidance system** to monitor and avoid frontal collision.
1.  We will be using the **collision_avoidance_system**  for monitoring and controlling the vehicle to avoid frontal crash with vehicl and pedestrians.
    To initiate the collision_monitor_control, please run the following command from a new terminal (terminal 5) as follows:

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
    ros2 run collision_avoidance_system collision_monitor_control
    ```

 
### 6. 🎯 Use Navigation HMI to select the destination location on the map.
1.  **Source the navigation package.**
    Open a new terminal (terminal 6) and run the following command:

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


## Alternatively RUNNING ALL above ROS nodes(steps 2 - step 6)  together from a single terminal with one launch file.
1.  Run a single launch file to run all above nodes together.
    In terminal 2, run the following command in a new terminal (terminal 7):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    source $HOME/CARLA_PROJECT/ros2_bridge_ws/ros-bridge/install/setup.bash
    export CARLA_ROOT=$HOME/CARLA_PROJECT/CARLA_0.9.15
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
    source $HOME/CARLA_PROJECT/carla_ros2_ws/install/setup.bash
    ros2 launch task_launch master_launch.py
    ```

### 7. 🎯 Use Rviz2 to select the destination location on the map.
1.  **Launch RViz 2** in a new terminal (terminal 8):

    ```bash
    conda deactivate
    source /opt/ros/humble/setup.bash
    rviz2
    ```

2.  Select the `carla_road_network` topic to **visualize the road network** in RViz 2.
3.  Select the `carla/hero/marker` topic to **visualize the vehicle** in RViz 2.
4.  Now choose the **"Goal Pose" option** to select the destination point in the road network.
5.  Once you select the goal pose, you can **see the new route created by the waypoint publisher** on the map.

### 8. 🎯 Use **CARLA pythonapi example** scripts to launch another vehicle in the simulator and test the behaviour of ego vehicle. 
1.  Open a new terminal and create a conda environment for running carla python example script with following command:

    ```bash
    cd cd $HOME/CARLA_PROJECT/carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2
    conda env create -f conda_carla_env.yml
    ```
2.  A new conda environment will be created with name 'carla_37'
3.  Then activate this environment, and run the 'manual_control.py' script file from CARLA executable folder.

    ```bash
    conda activate carla_37
    cd $HOME/CARLA_PROJECT/CARLA_0.9.15/PythonAPI/examples
    python3 manual_control.py --rolename vehicle
    ```

2.  A new pygame window appear to control the new vehicle in the simulator.
3.  You can drive this car manually infront of the EGO VEHICLE and see how ego vehicle control its speed when a collision detected.
4.  Also you can keep this vehicle infront of EGO VEHICLE and keep it in Autopilot mode (by pressing 'p' KEY), so that you can see the ego vehicle adjust its according to second vehicle.
5.  Also you can manually drive second vehicle to craete a collision case and see how EGO vehicle react to it. 

### Video tutorial
https://youtu.be/HqB76WQdf8g

