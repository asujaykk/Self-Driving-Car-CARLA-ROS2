import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import carla
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, DurabilityPolicy

class CarlaMapPublisher(Node):
    def __init__(self):
        super().__init__('Carla_navigation_hmi')

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        
        #enable speed command 
        self.declare_parameter('enable_spd_cmd', True)
        self.enable_spd_cmd = self.get_parameter('enable_spd_cmd').get_parameter_value().bool_value
        
        # Generate waypoints every 2 meters
        self.waypoints = self.map.generate_waypoints(0.7)

        self.publisher_ = self.create_publisher(MarkerArray, 'carla_road_network', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, "/carla/hero/goal_pose", 10)
        
        qos_profile = QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.control_speed_publisher = self.create_publisher(Float64, '/carla/hero/speed_command',qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.markers = self.create_markers(self.waypoints)

        self.init_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            qos_profile=10)
        
        self.init_pose_subscriber  # prevent unused variable warning

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            qos_profile=10)
        
        self.goal_pose_subscriber  # prevent unused variable warning


    def initial_pose_callback(self, msg):
         self.get_logger().info(f"Received Initial Pose: Position=({msg.pose.pose.position.x}, {msg.pose.pose.position.y}), Orientation=({msg.pose.pose.orientation.z}, {msg.pose.pose.orientation.w})")

    def goal_pose_callback(self, msg):
         self.get_logger().info(f"Received goal Pose: Position=({msg.pose.position.x}, {msg.pose.position.y}), Orientation=({msg.pose.orientation.z}, {msg.pose.orientation.w})")
         #only send speed command if speed command enabled
         speed_msg = Float64()
         #speed_msg.data = 20.0   # maintain 20km speed
         if self.enable_spd_cmd:
            speed_msg.data = 20.0
            self.control_speed_publisher.publish(speed_msg)  #send  speed of the vehicle to be maintained
            self.get_logger().info(f"published spd command 20" )
         else:
            speed_msg.data = 0.0
            self.control_speed_publisher.publish(speed_msg)  #send  speed of the vehicle to be maintained   
            self.get_logger().info(f"published spd command 0" )
                    
         self.goal_pose_publisher.publish(msg)  # send goal pose to waypoint publisher
         self.get_logger().info(f"published goal" )
 
    def create_markers(self, waypoints):
        marker_array = MarkerArray()
        for idx, wp in enumerate(waypoints):
            marker = self.create_marker(wp, idx)
            marker_array.markers.append(marker)
        return marker_array

    def create_marker(self, waypoint, idx):
        marker = carla_marker_to_ros_marker(waypoint, idx)
        return marker

    def timer_callback(self):
        self.publisher_.publish(self.markers)

def carla_marker_to_ros_marker(waypoint, idx):
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rclpy.clock.Clock().now().to_msg()
    marker.ns = "carla_waypoints"
    marker.id = idx
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x = waypoint.transform.location.x
    marker.pose.position.y = -(waypoint.transform.location.y)# -73.0)
    marker.pose.position.z = waypoint.transform.location.z *(-1)
    return marker

def main(args=None):
    rclpy.init(args=args)

    node = CarlaMapPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
