#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
import cv_bridge as CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from simple_pid import PID

## Class that subscribes to camera and moves the robot based on the PID controller.
#
class move_robot:
    ## Constructor for the class
    def __init__(self):
        # Subscribe to the camera and initialize the velocity publisher and bridge
        self.bridge = CvBridge.CvBridge()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.prevCenter = 0
        self.pid = PID(1, 0.2, 0.01, setpoint = 0)
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback)

    ## Callback function for the subscriber.
    #
    #  @param data is the image data received by the subscriber.
    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = self.pid(self.getPidError(frame))
            self.publisher.publish(twist)
        except self.CvBridgeError as e:
            print(e)

    ## Function that returns the error for the PID controller.
    #
    #  @param img is the input image for the function.
    #
    #  @returns the error value for the PID controller.
    def getPidError(self,img):
        height, width, _ = img.shape
        imageCenter = width // 2

        x = np.where(img[height-20, :, 1]<100)[0]
  
        if(len(x) == 0):
            cX=0
            if(self.prevCenter<width//2):
                cX= self.prevCenter-1
            elif(self.prevCenter>width//2):
                cX= self.prevCenter+1
        else:
            cX = (x[0]+x[-1])//2

        self.prevCenter = cX
        cv2.circle(img, (cX, height-30), 10, (0, 0, 255), -1)
        cv2.imshow("Camera_View", img)
        cv2.waitKey(3)
        print(imageCenter-cX)
        return -(imageCenter-cX)/500


    

if __name__ == '__main__':
    try:
        robot = move_robot()
        rospy.init_node('topic_publisher')
        # Execute file until stopped by a keyboard interrupt. 
        rospy.spin()
    except(KeyboardInterrupt):
        print("Shutting off simulation.")
    cv2.destroyAllWindows()
