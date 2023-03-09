import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from time import sleep

from sensor_msgs.msg import CompressedImage

class image_buffer:
    def __init__(self):
        print("initializing")
        self.image_pub = rospy.Publisher("/buffer/cam", CompressedImage, queue_size = 1)
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        print("initialized")
    def callback(self, ros_data):
        self.image_pub.publish(ros_data)

def main(args):
    '''Initializes and cleanup ros node'''

    rospy.init_node('image_buffer', anonymous=True)
    ib = image_buffer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
