# ==============================================================================
# -- sarround interface-----------------------------------------------------------
# ==============================================================================

import shapely as shply
from shapely import LineString, Point, Polygon , LinearRing

import cv2
import numpy as np
import sys

class SarroundMonitor():
    
   def __init__(self,speed_limit):
        #list of objects to be avoided colliding with.
        car=(0,0,142)
        truck = (0, 0, 70)
        bus = (0, 60, 100)
        mcycle = (0, 0, 230)
        cycle = (119, 11, 32)
        Rider = (255, 0, 0)
        pedestrian=(220,20,60)
        self.objects={car,truck,bus,mcycle,cycle,Rider,pedestrian }  
     
        self.monitor_zone = None
        self.zone_point = None

        self.spd_at_full_zone=speed_limit
        self.img_scale =0.7      
        
        p_bl= (320,720)
        p_br= (960,720)
        p_tl= (630,400)
        p_tr= (650,400)
    
        #first set degfult zone , can be used to configure zone based on image size
        self.set_mone_zone()


   def set_mone_zone(self,ytop=400,ybot=720,xbl=320,xbr=960,xtl=630,xtr=650):
        
        self.zone_point = (
            int(ytop*self.img_scale),
            int(ybot*self.img_scale),
        
            int(xbl*self.img_scale),
            int(xbr*self.img_scale),
        
            int(xtl*self.img_scale),
            int(xtr*self.img_scale),
        )
        
   def update_zone_area(self,percent=10):    
        
        ytop,ybot,xbl,xbr,xtl,xtr = self.zone_point
        
        def get_left_line_x(y):
           return int ((((y-ytop)*(xbl-xtl))/(ybot-ytop))+xtl)
        
        def get_right_line_x(y):
           return int((((y-ytop)*(xbr-xtr))/(ybot-ytop))+xtr)
        
        n_ytop=int( ybot- (((ybot-ytop)/100.00)*percent))
        
        n_xtl= get_left_line_x (n_ytop)
        n_xtr= get_right_line_x (n_ytop)
        
        self.monitor_zone=np.array([[xbl, ybot], [n_xtl,n_ytop],
                        [n_xtr,n_ytop], [xbr,ybot]],
                       np.int32)

        
   def conv_to_shp(self,pts):
      return np.reshape(pts,(pts.shape[0],2))

   def get_object_mask(self,img,obj):
      r_t =img[:,:,0] == obj[2]
      g_t= img[:,:,1]== obj[1]
      b_t= img[:,:,2]== obj[0]
      
      b_a=np.logical_and(r_t,g_t)
      b_a=np.logical_and(b_a,b_t)

      return  b_a

   def get_object_polygons(self,mask):
       contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
       poly=[]
       for cnt in contours:
          approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
          (x,y)=cnt[0,0]
          
          if len(approx) >= 5:
             poly_pt=self.conv_to_shp(approx)
             #print(approx)
             #print(conv_to_shp(approx))
             #obj = Polygon(poly_pt)
             poly.append(poly_pt)
       
       return poly      
             
   def is_object_in_danger_zone(self,imgseg,depth_mask, imgrgb):

        mask=np.full_like(imgseg[:,:,0],False, dtype=np.bool_)
        for obj in self.objects:
            mask=np.logical_or(mask,self.get_object_mask(imgseg,obj))
        
        depth_intersect_mask= np.logical_and( mask,depth_mask)
        
        #used for emergency breaking
        #close_distance = depth_mask[depth_intersect_mask].min()
        
        mask=mask.astype(np.uint8)
        poly_list=self.get_object_polygons(mask) 
        
        shp_zone=Polygon(self.monitor_zone)
        
        img =imgrgb
        danger_status = False
        object_in_col_zone = False
        for ply in poly_list:
            shp_ply=Polygon(ply)
            if not shp_ply.is_valid:
               # Attempt to clean up the polygon using a zero-width buffer
               # to correct shapely.errors.GEOSException: TopologyException: side location conflict at 422 148. 
               # This can occur if the input geometry is invalid.
               shp_ply = shp_ply.buffer(0)
               
               
            if shply.intersects(shp_zone,shp_ply):
                img = cv2.polylines(img, [ply], True, (0,0,255), 2)
                object_in_col_zone =True
                

        
        # Create a mask for the polygon
        mask = np.zeros_like(imgrgb[:,:,0], dtype=np.uint8)
        
        #craee polygon points
        points =self.monitor_zone.reshape((-1, 1, 2))
        
        # Draw the polygon onto the mask
        cv2.fillPoly(mask, [points], color=True)

        out_mask = np.logical_and( mask,depth_intersect_mask)
        
        danger_status = np.any(out_mask)
        
        return danger_status,object_in_col_zone, img 


   def Alert(self,is_danger,object_in_col_zone,imgrgb,spd):
       if imgrgb is not None:
          img=imgrgb
          img_h,img_w,_=img.shape

          pts = self.monitor_zone.reshape((-1, 1, 2))
                   
          img[10:150,10:300,:] = 255
          
          isClosed = True
          
          # Blue color in BGR
          line_color = (0, 255, 0)
           
          # Line thickness of 2 px
          thickness = 2
           
          # Using cv2.polylines() method
          # Draw a Blue polygon with
          # thickness of 1 px
          if object_in_col_zone:
             # Blue color in BGR
             line_color = (0, 0, 255)
             img = cv2.rectangle(img,(300,10) , (500,100), (0,100,250), -1)
             img=cv2.putText(img,"AEB ALERT..", (325,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
             
             img=cv2.putText(img,"SPEED: "+str(spd)+" km/h", (25,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
             img=cv2.putText(img, "Collision alert..!!!", (25,65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
        

             if is_danger :
                 # Blue color in BGR
                 line_color = (0, 0, 255)
                 img = cv2.rectangle(img,(300,10) , (500,100), (0,50,250), -1)
                 img=cv2.putText(img,"AEB ENGAGING..", (325,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
                 img=cv2.putText(img,"AUTO BRAKING..", (325,70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)    
                  
                 img=cv2.putText(img, "Overriding vehicle control..", (25,105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
                 img=cv2.putText(img, "AEB engaging..",(25,125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1) 

                 
             
          else:
              img = cv2.rectangle(img,(300,10) , (500,100), (0,255,0), -1)
              img=cv2.putText(img,"AEB MONITORING..", (325,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
              
              img=cv2.putText(img,"SPEED: "+str(spd)+" km/h", (25,25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
              img=cv2.putText(img, "Driver controlling.. ", (25,65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
          
          img = cv2.polylines(img, [pts],isClosed, line_color, thickness)  
          
          cv2.imshow('detection',img)
          cv2.waitKey(1)
   
   def  get_near_obj(self,depth_img,distance=100):
       """
        The distance map from depth image is calculated with the following equations.
        normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        in_meters = 1000 * normalized
       """
       
       #tm=0.000059605
       #bm=65536.00
       #depth_meter_map= (depth_img[:,:,2]+(depth_img[:,:,1]*256)+ (depth_img[:,:,0]*bm))*tm
       
       
       return depth_img < distance

   def predict_stopping_distance(self,spd): #spd in km/h
       #stoping_distance = v**/(2a) # a decelaration of vehicle  (common value: 7-10m/s**) 
       #spd to m/s =>  then v**/2a => (spd*spd) * 0.0771
       # stopping_distance = (spd*spd ) * 0.0771/(2*9)     (lets a = 9 m/s**)
       
       stopping_distance= ((spd * spd) * 0.004286) + (spd*0.3)  #additional distance for sensor + actuator latency
       return max(3,stopping_distance)  #keep 2 meter as minimum distnace from camera
       
   def monitor_and_alert(self,sensr_data): #SnsrInterface,CntrlInterface):
        
        #imgs,spd=SnsrInterface.getSensorData()
        imgs,spd = sensr_data

        imgrgb,imgdep,imgseg = imgs
        if imgseg is None:
            return 
        imgrgb = cv2.resize(imgrgb, None, fx=self.img_scale, fy=self.img_scale, interpolation=cv2.INTER_LINEAR)
        imgdep = cv2.resize(imgdep, None, fx=self.img_scale, fy=self.img_scale, interpolation=cv2.INTER_LINEAR)
        imgseg = cv2.resize(imgseg, None, fx=self.img_scale, fy=self.img_scale, interpolation=cv2.INTER_LINEAR)
        
        precentage=int(min(100.0,max(10,(spd/self.spd_at_full_zone)*100.0)))
        
        self.update_zone_area(percent=precentage)
        
        stopping_distance = self.predict_stopping_distance(spd)
        
        #print(stopping_distance)
        mask=self.get_near_obj(imgdep,distance=stopping_distance)
        stat,object_in_col_zone, img_f= self.is_object_in_danger_zone(imgseg,mask,imgrgb) 

        self.Alert(stat,object_in_col_zone, imgrgb,int(spd))
        
        #self.Alert(stat,object_in_col_zone, imgrgb,int(spd))
        return stat , object_in_col_zone

