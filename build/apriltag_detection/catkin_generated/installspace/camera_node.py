#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('usb_camera_publisher')
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass