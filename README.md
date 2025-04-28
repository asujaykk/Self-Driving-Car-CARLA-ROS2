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

So we are going to use CARLA SIMULATOR for this purpose, it is really heavy simulator (consider a performance laptop/workstation).
And we can read the sensor data from this vehicle with python API/ ROS bridge.
    
