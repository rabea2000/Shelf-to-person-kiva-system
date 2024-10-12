#!/usr/bin/env python3

import rospy 
import numpy as np
import math
import  tf 
from math import pi 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Point, Quaternion
import time 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image   
import cv2.aruco as aruco
import cv2  as cv
import localization 
from std_msgs.msg import Bool
from AStar.binary_map_4n   import *

class Controller :

    def __init__ (self) :
        self.mode = 'hardware'

        if self.mode == 'sim' :

            self.max_speed = 0.7   # for actule is 0.035
            self.max_angular_vel =0.4 # for actule is 0.1 
            self.steer_kp = 10
            self.speed_kp =0.8
            self.odom_topic_name = '/odom'


        else : 
            self.max_speed = 0.018 #0.5   # for actule is 0.035
            self.max_angular_vel = 0.04 #0.4 # for actule is 0.1 
        
            self.steer_kp = 0.18 #1.5
            
            self.speed_kp = 0.03
            self.speed_ki = 0.001

            self.odom_topic_name = '/vo'

        # self.max_speed = 0.5   # for actule is 0.035
        # self.max_angular_vel =0.4 # for actule is 0.1 
        
        self.dt = 10
        self.r  = rospy.Rate(1/self.dt) 
        self.start_x = None
        self.start_y = None
        self.flage = True
        self.current_time = 0 
        self.privous_time = 0
        self.camera_matrix = localization.Localization().camera_matrix 
        self.dis_matrix = localization.Localization().dis_matrix 
        self.aruco_size = 79.502
        self.limitSwitchState = False 
        self.start_x_local = 0
        self.start_y_local = 0


        rospy.Subscriber('/limitswitch', Bool, self.limitswitch)
        self.result_pub = rospy.Publisher('line/result' ,Image , queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

 
        
        
        points = [(0, 0), (1, 0), (1, 0.25), (1.4, 0.25), (1.4, 1)]

        # List comprehension to create a new list of dictionaries
        self.path  = [{'x': x/100, 'y': y/100} for x, y in points[1:]]

        


        self.goal =(200 ,100)

       
    
        
    def orientationError (self ,goal_x,goal_y , x, y , current_orientation) :
        orientation_goal  = math.atan2(goal_y - y , goal_x - x)
        
        orientation_error = orientation_goal - current_orientation 
        return orientation_goal , orientation_error  

 


    def positionError(self,goal_x,goal_y,current_x, current_y) :
        x = goal_x - current_x
        y = goal_y - current_y
        dist_error = math.sqrt(x**2 + y**2)

        if self.flage :
            self.start_x = current_x 
            self.start_y = current_y 
            self.flage = False  



        
        #that means the robot move along y axis
        if (abs(goal_x - self.start_x) < abs(goal_y - self.start_y)) :
            #rospy.loginfo("the robot move along y axis")
            if goal_y >  self.start_y:
                sign = self.sign(goal_y , current_y)
                
            # positive => backward , negative =>forward along x
            else  :
                sign = self.sign(current_y , goal_y)    
                
            return sign * dist_error
           

        #that means the robot move alonge x axis
        else : # (abs(goal_y - self.start_y) < 0.5):
            #rospy.loginfo("the robot move along x axis")
            
            # positive => forward , negative =>backward along x
            if goal_x >  self.start_x:
                sign = self.sign(goal_x , current_x)
                #rospy.loginfo(" from small to big ")
                
            # positive => backward , negative =>forward along x
            else  :
                sign = self.sign(current_x , goal_x) 
                #rospy.loginfo(" from big to small ")
            
            return sign * dist_error
        
        
    def positionErrorLocal(self,goal,current , point) :
        x = goal['x'] - current['x']
        y = goal['y'] - current['y']
        dist_error = math.sqrt(x**2 + y**2)


        #that means the robot move along y axis
        if (abs(goal['x']  - self.start_x) < abs(goal['y'] - self.start_y)) :
            #rospy.loginfo("the robot move along y axis")
            if goal['y'] > self.start_y:
                sign = self.sign(point['y'] , current['y'])
                
            # positive => backward , negative =>forward along x
            else  :
                sign = self.sign(current['y'], point['y'])    
                
            return sign * dist_error
           

        #that means the robot move alonge x axis
        else : # (abs(goal_y - self.start_y) < 0.5):
            #rospy.loginfo("the robot move along x axis")
            
            # positive => forward , negative =>backward along x
            if goal['x']  >  self.start_x:
                sign = self.sign(point['x'] , current['x'])
                #rospy.loginfo(" from small to big ")
                
            # positive => backward , negative =>forward along x
            else  :
                sign = self.sign(current['x'], point['x']) 
                #rospy.loginfo(" from big to small ")
            
            return sign * dist_error
        






    def rotateToGoal(self,goal = None , target_angle_enabled = False , target_angle = 0  ):
        command = Twist()
        odom_data = rospy.wait_for_message(self.odom_topic_name, Odometry)
        x, y , angle  = self.get_odom(odom_data) 

        if target_angle_enabled :
            desird_angle = math.radians(target_angle)

        else :     
            desird_angle  = math.atan2(goal['y']- y , goal['x'] - x )
        
        
        
        
        while  not rospy.is_shutdown()  :
            odom_data = rospy.wait_for_message(self.odom_topic_name, Odometry)
            _, _ , angle  = self.get_odom(odom_data) 

            error = self.rotation_error(desird_angle, angle)

            if abs(error) <= math.radians(1) :
                rospy.loginfo("we rotate to desired angel ")
                command.angular.z = 0 
                self.pub.publish(command) 

                
                break

            angluer_vel =  self.steer_kp * error 
            
            command.angular.z =  self.saturate(angluer_vel ,-self.max_angular_vel , self.max_angular_vel) 
             
            command.linear.x  =  0         
            self.pub.publish(command) 
            rospy.loginfo(f" current angle : {math.degrees(angle)} desird_angle:{math.degrees(desird_angle)}  error : {math.degrees(error)}  ")
            self.r.sleep 

        
    def goToPostione(self , goal ):
        command = Twist()
        self.flage = True

        odom_data = rospy.wait_for_message(self.odom_topic_name, Odometry)
        x, y , angle = self.get_odom(odom_data) 
        desird_angle_pre = math.atan2(goal['y']- y , goal['x'] - x )
        privouse_time = 0 
        integral  = 0 
        privouse_error = 0


        

        while  not rospy.is_shutdown() :
            odom_data = rospy.wait_for_message(self.odom_topic_name, Odometry)
            x, y , angle = self.get_odom(odom_data) 
            error =  self.positionError(goal['x'],goal['y'],x,y)
            
            # if abs(error) >  0.2:
            desird_angle  = math.atan2(goal['y']- y , goal['x'] - x )
            #     desird_angle_pre = desird_angle
            # else :
            #     desird_angle  =  desird_angle_pre

            # error =  self.positionError_normalLine(goal['x'],goal['y'],x,y)

            rospy.loginfo(f" goalx : {goal['x']} goaly : {goal['y']}  dist_error is {error}  x: {x} y :{y} desird angle {math.degrees(desird_angle)}")

            if abs(error) < 0.05:
                rospy.loginfo("we arrived to postion")
                command.linear.x  = 0
                command.angular.z = 0 
                self.pub.publish(command) 

                break 

            error_angluer = desird_angle - angle
            current_time = time.time() 
            delta_time = current_time - privouse_time 
            print(f"delta_time : {delta_time}")
           
            integral += error_angluer * delta_time
            e_D = (error_angluer - privouse_error)/delta_time
        

            vel = 0.05 * error 

            privouse_time = current_time 
            privouse_error =error_angluer

            command.linear.x  =  self.saturate(vel,-self.max_speed , self.max_speed)

            # error_angluer = self.rotation_error(desird_angle, angle) 
                       
            angluer_vel  =   0.5 * error_angluer + 0 *integral + e_D *0.01
            command.angular.z =  self.saturate(angluer_vel ,-self.max_angular_vel , self.max_angular_vel) 
           
    
            

            if abs(desird_angle - angle) >= math.radians(90) :
            # when we are close to overshoot we want to stop the rotation control
                command.angular.z = 0 

            if abs(error) < 0.08:
                # rospy.loginfo("we arrived to postion")
                # command.linear.x  = 0
                command.angular.z = 0 
                

                
           
            self.pub.publish(command) 
            
            self.r.sleep 
        

    def goToGoal(self) :  

        # self.goal= (115, 92)
        odom_data = rospy.wait_for_message(self.odom_topic_name, Odometry)
        x, y , angle = self.get_odom(odom_data) 
        # path , _ = path_planing((int(x*100), int(y*100)) , (self.goal))
        path = [(0, 0), (115, 0), (115, 92),(184 ,92)]
        self.publish_path(path)
        self.path  = [{'x': x/100, 'y': y/100} for x, y in path[1:]]
        
        print(self.path)
        
        for  goal in self.path :
            rospy.loginfo(f"the new pos is x:{goal['x']} y :{goal['y']}")
            rospy.loginfo(f"rotate")
            self.rotateToGoal(goal)  
            # rospy.sleep()
            
            self.goToPostione(goal)

        print(self.path) 
        rospy.sleep(2)   


        reverse_path = path[::-1] 
        reverse_path  = [{'x': x/100, 'y': y/100} for x, y in reverse_path[1:]]
        print(f"the reversed path is{reverse_path}")   
        
        for  goal in reverse_path : 
            rospy.loginfo(f"the new pos is x:{goal['x']} y :{goal['y']}")
            rospy.loginfo(f"rotate")
            self.rotateToGoal(goal)  
            
            
            self.goToPostione(goal)  

             
                
        print(f"the reversed path is{reverse_path}")  

        print(f"the path {self.path} ")    
        # i = 0



 

 



    def limitswitch (slef , msg:Bool) :
        slef.limitSwitchState = True 
                

    def publish_path (self , paths) :
        pub = rospy.Publisher("/path", Odometry, queue_size=10)

        rospy.sleep(1)
        for path in paths :
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = round(path[0]/100 , 3)
            odom.pose.pose.position.y = round(path[1]/100 , 3)
            pub.publish(odom)
        

                       

    
    def sign (self, num1 , num2) :
        result = num1 - num2 
        if (result < 0) :
            return -1 
        else :
            return 1



    def get_odom( self ,msg : Odometry) :

        robot_x = msg.pose.pose.position.x 
        robot_y = msg.pose.pose.position.y 

        if self.mode == 'sim' :
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        else :
            yaw     = msg.pose.pose.orientation.z  # comment


        
        return robot_x ,robot_y ,  yaw 
    
    def saturate(self,value, min, max):
        if value <= min: return(min)
        elif value >= max: return(max)
        else: return(value)
    

    def rotation_error (self ,desird_angle, angle):
        

        error  = desird_angle - angle
        if error  > pi:
                error -= 2*pi
        elif error < -pi:
                error  += 2*pi
        

        return   error      

    def scale(self , angle):
        res = angle
        
        if angle < 0:
            return res + (2 * math.pi)
        return res
    
    def normalize( self , angle):
        res = angle
        while res > pi: 
            res -= 2.0 * pi 
        while res < -pi: 
            res += 2.0 * pi 
        return res     




if __name__ == "__main__" :
    print("start")
    rospy.init_node("controller")
    controller = Controller() ; 
    controller.goToGoal()







#    def find_normal_line(self ,m, b, x1, y1):
  
#         normal_slope = -1/m
  
#         b = y1 - normal_slope *x1
#         return normal_slope ,b 

#     def positionError_normalLine(self,goal_x,goal_y,current_x, current_y) :
#         x = goal_x - current_x
#         y = goal_y - current_y
#         dist_error = math.sqrt(x**2 + y**2)

#         if self.flage :
#             self.start_x = current_x 
#             self.start_y = current_y 
#             self.flage = False  

#         angle = math.atan2((goal_y - self.start_y) ,(goal_x - self.start_x ))
        
#         if (goal_x - self.start_x) == 0  :
#             # print(f"the robot move on y axis where the x equle eche other and the normil y = {goal_y} horizontal")
#             print("move on y axis")
 
#             if  angle  == pi/2  : 
                
#                 if current_y > goal_y :    
#                     # print(f"the point above ")
#                     return -dist_error
#                 else :
#                     # print(f"the point under  ")
#                     return dist_error
                
                
#             else : 
#                 if current_y > goal_y :    
#                     # print(f"the point under ")
#                     return dist_error
#                 else :
#                     # print(f"the point above  ")
#                     return -dist_error
                
#         elif (goal_y - self.start_y) == 0 :
#             print("move on x axis")

#             # print(f"the robot move on x axis where the y equle eche other and the normil x = {goal_x} vertical")


#             if  angle  == 0 : 
#                 if current_x > goal_x :    
#                     # print(f"the point above ")
#                     return -dist_error
#                 else :
#                     # print(f"the point under  ")
#                     return dist_error
                
#             else : # 
#                 if current_x > goal_x :    
#                     # print(f"the point under ")
#                     return dist_error
#                 else :
#                     # print(f"the point above  ")
#                     return -dist_error
#         else:
#             print("move to linear")
#             m = (goal_y - self.start_y) /  (goal_x - self.start_x)
#             y_intercept = goal_y - m*goal_x

#             normal_line_slop ,y_intercept_normal_line = self.find_normal_line(m ,y_intercept , goal_x , goal_y)
            

#             y_normal = normal_line_slop * current_x + (current_y -normal_line_slop*current_x)

#             if angle > 0:
#                 print(f"angle > 0 )current_Y{current_y} and y normal {y_normal}")
#                 if  current_y  > y_normal  : 
#                     print(f"the point above the normal  ")
#                     return -dist_error
#                 else :
#                     print(f"the point under the normal ")
#                     return dist_error
                    
#             else : 
#                 if  current_y  > y_normal  : 
#                     print(f"the point under the normal  ")
#                     return dist_error
#                 else :
#                     print(f"the point above the normal ")
#                     return -dist_error
                    


 
    



