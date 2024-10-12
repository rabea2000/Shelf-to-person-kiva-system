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
from std_msgs.msg import Int32 , Bool
from fixedList import FixedList 
from geometry_msgs.msg import Twist


aruco_pos = {

    17: (0, 184),
    18: (46, 184),
    19: (92, 184),
    20: (138, 184),
    21: (184, 184),
    22: (227, 184),

    23: (0, 138),
    24: (46, 138),
    25: (92, 138),
    26: (138, 138),
    27: (184, 138),
    28: (227, 138),

    29: (0, 92),
    30: (46, 92),
    31: (92, 92),
    32: (138, 92),
    33: (184, 92),
    34: (227, 92),

    35: (0, 46),
    36: (46, 46),
    37: (92, 46),
    38: (138, 46),
    39: (184, 46),
    40: (227, 46),

    41: (0 , 0),
    42: (46 ,0),
    49: (92,0),
    44: (138, 0),
    45: (184, 0),
    46: (227, 0)
}

class Localization :

    def __init__(self) :
       
        self.camera_matrix  = np.array( [[511.12578234  , 0.  ,       327.1777829 ],
                        [  0.   ,      511.55405549, 239.64880834],
                        [  0.      ,     0.       ,    1.        ]])

        self.dis_matrix  = np.array( [[ 2.01964631e-01 ,-9.74963504e-01 , 5.26468449e-04 , 1.79351186e-03,
        1.36063603e+00]])


        # self.camera_sim_matrix = np.array ([[381.36246688113556, 0.0, 320.5],
        #                        [0.0, 381.36246688113556, 240.5],
        #                        [0.0,     0.0,      1.0]   ])

        # self.dis_sim_matrix = np.array ([[0.0, 0.0, 0.0, 0.0, 0.0]])


        self.aruco_size = 79.502 #milimeter in real 
        # self.aruco_size = 110 #milimeter in sim
        self.tankmode = False 
        self.arcedmode = False 
        self.movementmode = True # true means arcedmode and false means tank mode

        self.x = 0 
        self.y  = 0
        self.angle = 0 


        self.arucox = 0 
        self.arucoy  = 0
        self.arucoangle = 0 

        self.odomx = 0 
        self.odomy  = 0
        self.odomangle = 0 
        
        self.count = 0 

        self.R  =  0.085/2  
        self.x_intia  = 0
        self.y_intia  = 0
        self.angle_intia  = 0
        
        
        self.L  =  0.221
        self.degreePerTick = 0.9 #degree
        self.ticksPerRev = 400    # count of ticks to compulte on revoulation

        self.xr_c = 0.136 
        self.yr_c = 0.07
        
        self.windowSize = 7
        self.windowx     =  FixedList(self.windowSize)
        self.windowy     =  FixedList(self.windowSize)
        self.windowangle =  FixedList(self.windowSize)


        self.meters_per_tick = (2 * math.pi * self.R) / (self.ticksPerRev)
        
           
        self.er_count  =  0
        self.el_count  =  0

        self.last_er_count  =  0
        self.last_el_count  =  0

        rospy.Subscriber('/encoder_right', Int32, self.encoder_right)
        rospy.Subscriber('/encoder_left', Int32, self.encoder_left)
        self.reset_endcoder_pub =  rospy.Publisher('/reset_encoder' ,Bool , queue_size=10)


        # self.pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        self.localiztionodom_pub = rospy.Publisher('/vo_odom' ,Odometry, queue_size=10)
        self.localiztionaruco_pub = rospy.Publisher('/vo_aruco' ,Odometry, queue_size=10)
        self.localiztion_pub = rospy.Publisher('/vo' ,Odometry, queue_size=10)


        self.result_pub = rospy.Publisher('web_camera/result' ,Image , queue_size=10)




    def callback(self , data):
        
        br = CvBridge()
        
        
        # rospy.loginfo("receiving video frame")
   
        # Convert ROS Image message to OpenCV image
        img = br.imgmsg_to_cv2(data)
        imggray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        (corners, ids, rejected) = aruco.detectMarkers(imggray, arucoDict ,self.camera_matrix , self.dis_matrix)
        
        # ids= None
        if ids is not None:
            aruco.drawDetectedMarkers(img, corners)
            rvec_all, tvec_all, obk_v = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.dis_matrix)

            postione = [] 

            for marker in range(len(ids)) :
                aruco.drawAxis(img, self.camera_matrix, self.dis_matrix, rvec_all[marker], tvec_all[marker], self.aruco_size)
                
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

            # self.windowangle.append(self.angle)
            # self.angle = self.medine_filter(self.windowangle)
            
            self.arucox , self.arucoy = self.robot_pos(self.arucox ,self.arucoy , self.arucoangle)


            # with  fileter 
            self.windowx.append(self.arucox)   
            self.windowy.append(self.arucoy)   
            self.windowangle.append(self.arucoangle) 




            self.arucox     = self.medine_filter(self.windowx)
            self.arucoy     = self.medine_filter(self.windowy)
            self.arucoangle = self.medine_filter(self.windowangle)



            aruco_filter = Odometry()
            aruco_filter.header.frame_id = "odom"
            aruco_filter.pose.pose.position.x = round(self.arucox , 3)
            aruco_filter.pose.pose.position.y = round(self.arucoy , 3)
            aruco_filter.pose.pose.orientation.z = self.arucoangle
            self.localiztionaruco_pub.publish(aruco_filter)

            rospy.loginfo(f"x_window filter {self.windowx}")
        

            # these lines of code to get the intia value for odom
            if self.count == 0 :
                self.odomx = self.arucox
                self.odomy = self.arucoy
                self.odomangle= self.arucoangle
            
                self.count += 1

            
            # print(f"finall id: {marker_id }  x: { self.x } y: {self.y}  angle {self.angle}")
 
        
                
        if self.count > 0 :
            
            # these lines of code that correect the drift of odom
            self.count += 1 
            if self.count  > 100 :
                if ids is not None: 
                    self.odomx = self.arucox
                    self.odomy = self.arucoy
                    self.odomangle= self.arucoangle
                    self.count +=1


            
            dr , dl ,dc = self.delta_pos()
            if self.movementmode :             # true means arcade mode 
                theta_dt = (dr - dl) / self.L
                rospy.loginfo("aracde mode")  
            else :                             # tank mode
                theta_dt = -1*(dr - dl) / self.L
                rospy.loginfo("tanke mode") 


            x_dt = dc * math.cos(self.odomangle +(theta_dt/2))
            y_dt =  dc * math.sin(self.odomangle +(theta_dt/2))
            # print(f"dr  {round(dr - dl , 6)} ")

            self.odomx = round(self.odomx + x_dt, 4)
            self.odomy = round(self.odomy + y_dt, 4)
            self.odomangle= round(self.normalize(self.odomangle + theta_dt ), 4)
            
            
            self.update_last() 

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = round(self.odomx , 3)
            odom.pose.pose.position.y = round(self.odomy , 3)
            odom.pose.pose.orientation.z = self.odomangle 
            self.localiztionodom_pub.publish(odom)



            

        if ids is not None: 
            self.x = self.arucox
            self.y = self.arucoy
            self.angle = self.arucoangle

        else :
            self.x = self.odomx
            self.y = self.odomy
            self.angle = self.odomangle


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

        # 7cm  
        # 11.5+ 6 cos45 = 13.6 cm 
        x = -(math.cos(angle)*self.xr_c - math.sin(angle)*self.yr_c ) + x_camera 
        y = -(math.sin(angle)*self.xr_c + math.cos(angle)*self.yr_c) + y_camera 

        return round(x, 4) , round(y,4) 

    def get_pos_marker (self , id) :
        if id in aruco_pos :
            return aruco_pos[id]  

        rospy.logerr('the marker not listed')


    
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
    

    def encoder_right(self , msg:Int32) : 
        self.er_count = msg.data 

    def encoder_left(self , msg:Int32) : 
        self.el_count = msg.data     

    def update_last(self) :
        rospy.loginfo("er :"+str(self.er_count)  +" el :" + str(self.el_count) )
        self.last_er_count = self.er_count 
        self.last_el_count = self.el_count 


    

    def delta_pos(self) :

        delta_er =   self.er_count -   self.last_er_count 
        delta_el =   self.el_count -   self.last_el_count
        # print(f"delta_er is {delta_er} and delta_el {delta_el}")

        Dr = self.meters_per_tick * delta_er    
        Dl = self.meters_per_tick * delta_el   
        
        
        Dc = (Dr + Dl) / 2
        return Dr ,Dl ,Dc 
    

    def reset_endcoder (self)  :
        msg = bool()
        
        self.reset_endcoder_pub.publish(msg)
        rospy.loginfo(F"reset er : {self.er_count}  el : { self.el_count}")

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


    def tankOrArcad (self,cmd_vel:Twist) :
            
        
         if cmd_vel.linear.x == 0 or  cmd_vel.angular.z  == 0 :
             self.movementmode = True # arcada mode 
         else :
             self.movementmode = False #tank    

         

if __name__ =="__main__":
    rospy.init_node("localization" , anonymous=True)
    pos = Localization() 
    rospy.sleep(1)
    pos.reset_endcoder()
    rospy.sleep(1)

    rospy.Subscriber('spcbot/camera/image_raw', Image, pos.callback)
    rospy.Subscriber('/cmd_vel', Twist, pos.tankOrArcad )
    
    rospy.spin()
    cv.destroyAllWindows()
    
    # r = rospy.Rate(25)
    # while not  rospy.is_shutdown() :
    #     pos.callback() 
    #     r.sleep()














    # odom.header.frame_id = "odom"
    # odom.header.stamp =rospy.Time.now() 
    # odom.pose.pose.position.x = round(self.x , 3)
    # odom.pose.pose.position.y = round(self.y , 3)
    # odom.pose.pose.position.z = 0.27

    # quat = self.quaternion(self.angle)
    # odom.pose.pose.orientation.x = quat[0]
    # odom.pose.pose.orientation.y= quat[1]
    # odom.pose.pose.orientation.z = quat[2]
    # odom.pose.pose.orientation.w = quat[3]
    # odom.pose.covariance = [0.05, 0.05, 0.0,  0.0, 0.0, 0.0, math.radians(1), 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
    # self.localiztion_pub.publish(odom)
