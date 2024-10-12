#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2  as cv
import cv2.aruco as aruco
import numpy as np
import math
from math import pi 
import tf 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry 







class Localization :

    def __init__(self) :
       

        self.camera_sim_matrix = np.array ([[381.36246688113556, 0.0, 320.5],
                               [0.0, 381.36246688113556, 240.5],
                               [0.0,     0.0,      1.0]   ])

        self.dis_sim_matrix = np.array ([[0.0, 0.0, 0.0, 0.0, 0.0]])


        self.aruco_size = 79.502 #milimeter in real 


        self.x = 0 
        self.y  = 0
        self.angle = 0 


        self.arucox = 0 
        self.arucoy  = 0
        self.arucoangle = 0 


        
        self.count = 0 

 
        
        
    
   
        self.xr_c = 0.136 
        self.yr_c = 0.07
        
        

        
        self.localiztion_pub = rospy.Publisher('/vo' ,Odometry, queue_size=10)


        self.result_pub = rospy.Publisher('web_camera/result' ,Image , queue_size=10)




    def callback(self , data):
        
        br = CvBridge()
        
        
        # rospy.loginfo("receiving video frame")
   
        # Convert ROS Image message to OpenCV image
        img = br.imgmsg_to_cv2(data)
        imggray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        (corners, ids, rejected) = aruco.detectMarkers(imggray, arucoDict ,self.camera_sim_matrix , self.dis_sim_matrix)
        
        # ids= None
        if ids is not None:
            aruco.drawDetectedMarkers(img, corners)
            rvec_all, tvec_all, obk_v = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_sim_matrix, self.dis_sim_matrix)

            postione = [] 

            for marker in range(len(ids)) :
                aruco.drawAxis(img, self.camera_sim_matrix, self.dis_sim_matrix, rvec_all[marker], tvec_all[marker], self.aruco_size)
                
                rvec =  rvec_all[marker][0]   
                tvec =  tvec_all[marker][0]           

                rvec_flipped = rvec*-1
                tvec_flipped = tvec*-1

                rotation_matrix , jacobian  = cv.Rodrigues(rvec_flipped)
                realworld_tvec = np.dot(rotation_matrix  ,  tvec_flipped)

                pitch,roll,yaw = self.rotationMatrixToEulerAngles(rotation_matrix)
                # tvec_str = "x=%4.0f y=%4.0f dir=%4.0f"  %(realworld_tvec[0] , -realworld_tvec[1] , realworld_tvec[2])

                

                
                marker_id = ids[marker,0] 
                
                x_marker , y_marker  = self.get_pos_marker(marker_id)



                x_realted_aruco =  realworld_tvec[1]*0.001
                y_realted_aruco =  -realworld_tvec[0]*0.001

                id_string = " id :"+str(marker_id)+"  x_aruco : "+str(round(x_realted_aruco,2))+" y : "+str(round(y_realted_aruco,2) )
                cv.putText(img,id_string,(20,460),cv.FONT_HERSHEY_PLAIN,2,(0,0,255),2 ,cv.LINE_AA)
                
                x = x_realted_aruco + x_marker*0.01 
                y = y_realted_aruco + y_marker*0.01
                z =  0.27

                angle = yaw 
                # print(f"id: {marker_id }  x: { x } y: {y}  angle {angle}")

                postione.append([x,y, angle])
            

            self.arucox , self.arucoy, self.arucoangle  =  self.posFromlistOfPostion(postione)

            
            
            
            self.arucox , self.arucoy = self.robot_pos(self.arucox ,self.arucoy , self.arucoangle)


     
            self.x = self.arucox
            self.y = self.arucoy
            self.angle = self.arucoangle



            localization_msg = Odometry()
            localization_msg.header.frame_id = "odom"
            localization_msg.pose.pose.position.x = round(self.x , 3)      
            localization_msg.pose.pose.position.y = round(self.y , 3)        
            localization_msg.pose.pose.orientation.z = self.angle    
            self.localiztion_pub.publish(localization_msg)      
                 



            rospy.loginfo(#" marker id :"+ str(marker_id)+ 
                " x :" +str(self.x)#+ "  x_ralted : "+str(x_realted_aruco)+"  x_marker : "+str(x_marker*0.01) + '\n'
                +" y:"+str(self.y)#+ "   y_ralted  : "+str(y_realted_aruco)+" y_marker : "+str(y_marker*0.01) +'\n'
                    +   "  dir  :"+str(math.degrees(self.angle) ) +'\n')
            
        


        result_imag = br.cv2_to_imgmsg(img, encoding='bgr8')
                
        self.result_pub.publish(result_imag)




    
    def robot_pos (self,x_camera ,y_camera ,angle) :
        # this transform the postion to center of robot   
        # put the distanac of camera realtive to the center of robot  in x and y dirction 

        # 7cm  
        # 11.5+ 6 cos45 = 13.6 cm 
        x = -(math.cos(angle)*self.xr_c - math.sin(angle)*self.yr_c ) + x_camera 
        y = -(math.sin(angle)*self.xr_c + math.cos(angle)*self.yr_c) + y_camera 

        return round(x, 4) , round(y,4) 



    def get_pos_marker (self ,id , num_ar_x =10, num_ar_y=10 ) :
            id_n = id - 1 

            
            x = int (id_n / num_ar_y)
            y = id_n % num_ar_y 
            
            return x*100 , y*100    
    
    def normalize( self , angle):
        res = angle
        while res > pi: 
            res -= 2.0 * pi 
        while res < -pi: 
            res += 2.0 * pi 
        return res  

        # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self,R) :
    
        assert(self.isRotationMatrix(R))
    
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z]) 
    




    


    def quaternion(self ,angle) :
        return tf.transformations.quaternion_from_euler(0,0 ,angle )

    def posFromlistOfPostion(self , pos_list) :
        sum_x =0
        sum_y =0
        sum_angle =0 
        count = len(pos_list)

        for pos in pos_list :
            sum_x     += pos[0]
            sum_y     += pos[1]
            sum_angle += pos[2]

        x = sum_x / count 
        y = sum_y /count 
        angle = sum_angle /count  

        return x , y , angle  
    

    def medine_filter(self , window ) :
        if len(window) == self.windowSize: 
            sorted_window = sorted(window)
            return sorted_window[3] 
        else :
            # return
            return window[len[window]-1]
        
        
    def avarge_filter(self ,window )  : 
        sum=0
        for value in window  :
            sum  += value 

        return sum / len(window)   




if __name__ =="__main__":
    rospy.init_node("localization" , anonymous=True)
    pos = Localization() 
    rospy.loginfo("start")
    rospy.sleep(2)
  
  

    rospy.Subscriber('spcbot/camera/image_raw', Image, pos.callback)
    
    
    rospy.spin()
    cv.destroyAllWindows()
    
    # r = rospy.Rate(25)
    # while not  rospy.is_shutdown() :
    #     pos.callback() 
    #     r.sleep()










