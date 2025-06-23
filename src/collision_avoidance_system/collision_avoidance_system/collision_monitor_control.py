import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carla_msgs.msg import CarlaEgoVehicleStatus

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, Bool

import cv2
import numpy as np
from collision_avoidance_system.sarround_monitor import SarroundMonitor




class CollisionMonitorControl(Node):
    def __init__(self):
        super().__init__('collision_monitor_control')
        
        self.img_rgb = None
        self.img_seg = None
        self.img_dep = None
        self.spd = None
        self.speed_limit = 60.00 # 60 km/h
        
        #create speed command publisher
        qos_profile = QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control_speed_publisher = self.create_publisher(Float64, '/carla/hero/speed_command',qos_profile)


        #monitor and alert module
        self.monitor_module = SarroundMonitor(self.speed_limit-10)

        #cv bridge for msg to cv image conversion
        self.cvbridge = CvBridge()
        
        #create subscriber for speed
        self.subscription_spd =self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/hero/vehicle_status',
            self.vehicle_speed_callback,
            10
        )        

        #create subscriber for rgb_front_sensor imager
        self.subscription_rgb =self.create_subscription(
            Image,
            '/carla/hero/rgb_front/image',
            self.image_callback_rgb,
            10
        )

        #create subscriber for seg_front_sensor imager
        self.subscription_seg =self.create_subscription(
            Image,
            '/carla/hero/semantic_segmentation_front_normal/image',
            self.image_callback_seg,
            10
        )

        #create subscriber for depth_front_sensor imager
        self.subscription_dep =self.create_subscription(
            Image,
            '/carla/hero/depth_front_normal/image',
            self.image_callback_dep,
            10
        )

        #timer for monitoring
        self.timer =self.create_timer(0.1,self.timer_callback)

    #call back for timer
    def timer_callback(self,):
        if self.img_rgb is None or  self.img_seg is None or  self.img_dep is None:
            pass
        else:
            imgs = (self.img_rgb,self.img_dep,self.img_seg)
            collision_possible ,objects_in_colzone = self.monitor_module.monitor_and_alert((imgs,self.spd)) 
            
            speed_msg =Float64()
            #if object detected in colzone then reduce speed
            if objects_in_colzone:
                speed_msg.data = max(0.0,self.spd-1.0)

            else:
                speed_msg.data = min(self.speed_limit,self.spd+1.0)
            
            #if collision detected, then apply full break AEB
            if collision_possible:
                speed_msg.data =0.0  #come to a ful stop
                self.control_speed_publisher.publish(speed_msg)
            #else apply reduced speed
            else:
                self.control_speed_publisher.publish(speed_msg)


    #callback for vehicle speed
    def vehicle_speed_callback(self,msg):
        speed_mps = msg.velocity #speed in m/s
        speed_kph = speed_mps * 3.6  # speed in km/h
        self.spd = speed_kph

    #call back for rgb sensor 
    def image_callback_rgb(self,msg):
        try:
            cv_img = self.cvbridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            #cv2.imshow('rgb image',cv_img)
            #cv2.waitKey(1)
            self.img_rgb = cv_img
        except Exception  as e:
            print('error reading image'+str(e))


    #call back for seg sensor 
    def image_callback_seg(self,msg):
        try:
            cv_img = self.cvbridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            #cv2.imshow('seg image',cv_img)
            #cv2.waitKey(1)
            self.img_seg = cv_img
        except Exception  as e:
            print('error reading image'+str(e))


    #call back for depth sensor 
    def image_callback_dep(self,msg):
        try:
            cv_img = self.cvbridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
            depth_img = np.nan_to_num(cv_img,nan=0.0)
            #depth_norm =cv2.normalize(cv_img,None,0,255,cv2.NORM_MINMAX)
            #depth_norm = np.uint8(depth_norm)
            #cv2.imshow('dep image',depth_norm)
            #cv2.waitKey(1)
            self.img_dep = depth_img

        except Exception  as e:
            print('error reading image'+str(e))


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()