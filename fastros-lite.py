import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from fastcore.all import *
from fastdownload import download_url
from fastai.vision.all import *
from fastai.data.external import *
from time import sleep
import timm
from sensor_msgs.msg import CompressedImage

# https://answers.ros.org/question/330654/how-to-connect-remote-roscore-with-python-in-runtime/

VERBOSE=True

learn = load_learner('trashsort-lite.pkl')

processing = False

def predict_trash(img):
    color_coverted = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(color_coverted)
    what,_,probs = learn.predict(pil_image)#.reshape(192,192)
    print(f"Probs len: {len(probs)}")
    print(f"Probs: {probs}.")
    return what

class image_feature:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size = 1, buff_size=2**29)
        if VERBOSE :
            print("subscribed to /usb_cam/image_raw/compressed")

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #print(np_arr)
        #print(len(np_arr))
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        #print(len(image_np))
        print(predict_trash(image_np))
        #cv2.imshow('cv_img', image_np)
        #cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

        #self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
