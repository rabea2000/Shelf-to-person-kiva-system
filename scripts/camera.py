#!/usr/bin/env python3


import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np 


def publish_message():
 
  
  pub = rospy.Publisher("spcbot/camera/image_raw", Image, queue_size=10)
     
  
  rospy.init_node('web_cam', anonymous=True)
     
 
 
     

  br = CvBridge()
     
  # url = 'http://192.168.112.5:8080/video' 

  url = 'http://192.168.91.151:8080/video' 
  # url = 'http://192.168.91.5:8080/video
  cap = cv2.VideoCapture(url)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  rospy.loginfo("connect with camera succisfully")
  count =0 
  rate = rospy.Rate(25) # 10hz
  while not rospy.is_shutdown():
     
     
      ret, image = cap.read()
         
      if ret == True:
        

        ros_image = br.cv2_to_imgmsg(image, encoding='bgr8')
        pub.publish(ros_image)
        # rate.sleep 
        print(f"publish camera frame {count}")
        count +=1
      
        cv2.imshow("camera", image)
        cv2.waitKey(10)

      
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass



