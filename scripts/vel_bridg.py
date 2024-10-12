#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry 
from std_msgs.msg import  Int16
from geometry_msgs.msg import Twist

# this node  is made for arduino  so processing is done here 
# and send to arduon just the  velocity of wheele which is
# the pulses of the number stepper motor in sec 
# if this node not running nothing  go to arduino 
rightVel_pub = rospy.Publisher('/right_vel' ,Int16, queue_size=10)
leftVel_pub = rospy.Publisher('/left_vel' ,Int16, queue_size=10)
R  =  0.085/2  
L  =  0.221

def convertTwisToVel (cmd_vel:Twist) :
            
        
    vel_r =  (2 * cmd_vel.linear.x + cmd_vel.angular.z * L)/(2 * R) 
    vel_l =  (2 * cmd_vel.linear.x - cmd_vel.angular.z * L)/(2 * R) 

    left_steps  =  int(vel_l * (360 / 0.9))
    right_steps =  int(-vel_r * (360 / 0.9))

    rightVel_pub.publish(right_steps)
    leftVel_pub.publish(left_steps)





if __name__ =="__main__":
    rospy.init_node("odomBridg" , anonymous=True)


    rospy.Subscriber('/cmd_vel', Twist,  convertTwisToVel)
    
  
    rospy.spin()

    

