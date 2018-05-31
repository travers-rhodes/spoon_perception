#!/usr/bin/env python

import rospy
import rospkg
import cv2
import numpy as np
import yaml
import datetime
import errno
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from spoon_perception.srv import *

from matplotlib import pyplot as plt

"""
corr_pub = rospy.Publisher('/spoon_perception/correlation_distance', Float64, queue_size = 10)
chi_pub = rospy.Publisher('/spoon_perception/chisquared_distance', Float64, queue_size = 10)
inter_pub = rospy.Publisher('/spoon_perception/intersection_distance', Float64, queue_size = 10)
hell_pub = rospy.Publisher('/spoon_perception/hellinger_distance', Float64, queue_size= 10)
"""

"""
#OPENCV_METHODS = (
#        ("Correlation", cv2.HISTCMP_CORREL, corr_pub),
#        ("Chi-Squared", cv2.HISTCMP_CHISQR, chi_pub),
#        ("Intersection", cv2.HISTCMP_INTERSECT, inter_pub),
#        ("Hellinger", cv2.HISTCMP_BHATTACHARYYA, hell_pub))
"""
class SpoonPerception:
    """class that subscribes to a ros image topic and does the perception
    related with the spoon
    """

    def __init__(self,image_topic_name,th_detection=0.5,number_frames=5,
                 record=False, record_dir=None):
        """

        """
        self.last_image = None
        self.image_topic = None
        self.image_topic_name = image_topic_name
        self.bridge = CvBridge()
        self.spoon_histogram = None #this will be a dict of H S V histograms
        self.spoon_mask = None
        self.calibrated = False #flag to know if the detection is calibrated
        self.crop_image_pub = rospy.Publisher('/spoon_perception/masked_image',Image, queue_size=10)
        self.detect_object_srv = None

        self.record = record
        self.record_frame = 0
        self.record_dir = record_dir

        self.th_detection = th_detection # bellow this th we say that there is a object
        self.number_frames = number_frames # number of frames that it takes to make the decision
        return

    def startRetreivingImages(self):
        self.image_topic = rospy.Subscriber(self.image_topic_name, Image, self.receiveImage)

        return

    def startDummyDetectObjectSrv(self):
        self.detect_object_srv = rospy.Service('detect_object_spoon', ObjectSpoon, self.dummyDetectObject)

        return

    def startDetectObjectSrv(self):
        self.detect_object_srv = rospy.Service('detect_object_spoon',ObjectSpoon,self.detectObject)

        return

    def detectObject(self,req):
        distances = []
        response = False

        for i in range(self.number_frames):
            image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                          timeout=None)
            image = convertMsgImageCv(self.bridge, image_msg)
            histogram = getHSVHistogram(image, self.spoon_mask)
            d = cv2.compareHist(self.spoon_histogram['H'], histogram['H'],
                                cv2.HISTCMP_CORREL)
            distances.append(d)

        if (np.mean(distances) < 0.5) and (np.mean(distances) > -0.5):
            response = True

        if self.record and response:
            self.saveMaskImage(image_msg)

        return ObjectSpoonResponse(response)

    def dummyDetectObject(self, req):

        image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                          timeout=None)

        self.saveMaskImage(image_msg)

        response = True

        return ObjectSpoonResponse(response)

    def receiveImage(self,data):
        """ this method is the calback to the topic of the image, it converts
        the image to opencv image
        """

        cv_image = convertMsgImageCv(self.bridge,data)
        self.last_image = cv_image

        # check if there is something in the spoon - this thing has to be moved
        # from here
        if self.calibrated:
            #print('------------NEW IMAGE -----------------')
            #histogram = getHSVHistogram(self.last_image, self.spoon_mask)
            #for (method_name, method, pub) in OPENCV_METHODS:
               # d = cv2.compareHist(self.spoon_histogram['H'], histogram['H'], method)
               # pub.publish(float(d))
               # print('Method:', method_name, ' Distance:', d)
            #print('------------FINISH----------------------')

            image = cv2.bitwise_and(self.last_image,self.last_image,mask=self.spoon_mask)
            self.crop_image_pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8'))


        return

    def calibrationSpoon(self):
        """this function gets an image and retreive the histogram and mask for the spoon
        """
        # get the white sheet image for mask calibration
        image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                           timeout=None)
        image = convertMsgImageCv(self.bridge, image_msg)
        mask, th = retreiveMask(image)
        rospy.logwarn('Put a white sheet of paper in the background'
                                '--- Press ENTER to accept the image'
                                '--- Press ANY key to reject')
        # wait until accepted the image to retreive the mask
        while( not displayWait(mask,'Calibration') ):
            image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                           timeout=None)
            image = convertMsgImageCv(self.bridge, image_msg)
            mask, th = retreiveMask(image)
            rospy.logwarn('Put a white sheet of paper in the background'
                                '--- Press ENTER to accept the image'
                                '--- Press ANY key to reject')


        # get the histogram without sheet of paper there
        image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                           timeout=None)
        image = convertMsgImageCv(self.bridge, image_msg)
        image = cv2.bitwise_and(image,image,mask=mask)
        rospy.logwarn('Remove the  white sheet of paper from the background'
                                '--- Press ENTER to accept the image'
                                '--- Press ANY key to reject')
        while( not displayWait(image, 'Calibration') ):
            image_msg = rospy.wait_for_message(self.image_topic_name, Image,
                                           timeout=None)
            image = convertMsgImageCv(self.bridge, image_msg)
            image = cv2.bitwise_and(image,image,mask=mask)
            rospy.logwarn('Remove the  white sheet of paper from the background'
                                '--- Press ENTER to accept the image'
                                '--- Press ANY key to reject')

        histogram = getHSVHistogram(image, mask)

        self.spoon_histogram = histogram
        self.spoon_mask = mask

        #histogramHSVPlot(histogram)

        # this dont work... dont know why
        #data = {'Histogram':self.spoon_histogram, 'Mask':self.spoon_mask}
        #with open("calibrationSpoon.yml", "w") as f:
           #yaml.dump(data, f)
        self.calibrated = True
        return

    def saveMaskImage(self,image_msg):

        image = convertMsgImageCv(self.bridge, image_msg)
        image = cv2.bitwise_and(image, image, mask=self.spoon_mask)

        image_dir = self.record_dir + 'image_'  + str(self.record_frame) + '.png'
        print(image_dir)
        cv2.imwrite(image_dir, image)

        self.record_frame = self.record_frame+1

        return

def displayWait(image,window_name='default'):
    """Function that displays a image and wait until key pressed
        if key pressed is enter it returns true otherwise false
    """
    image_accepted = False

    cv2.imshow(window_name,image)
    k = cv2.waitKey(0)
    rospy.logwarn("you pressed key %s"% k)
    if k == 13 or k == 10: #enter key code
        image_accepted = True

    cv2.destroyAllWindows()
    return image_accepted


def convertMsgImageCv(bridge, ros_image_msg):
    """ function that given a cv bridge and a ros_image_msg converts the image to a opencv bgr8 image
    """
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image_msg,'bgr8')
    except CvBridgeError as e:
        rospy.logerr(e)
        cv_image = None

    return cv_image

def retreiveMask(image, intensity_th=0):
    """
    function return the image mask based on the threshold, the image
    should be in bgr8. Based in OTSU's method, returns the
    threshold calculated and the mask with the object
    """
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    th, mask  = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    return mask,th

def getHSVHistogram(image, mask=None):
    """
    function that gets the HSV histogram of a bgr8 image, optinaly can include
    a mask for the image
    returns in the form of a dictionary with H: S: V:
    """

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    hist_H = cv2.calcHist( [hsv_image], [0], mask, [180], [0, 179])
    hist_S = cv2.calcHist( [hsv_image], [1], mask, [256], [0, 255])
    hist_V = cv2.calcHist( [hsv_image], [2], mask, [256], [0, 255])

    histogram = {'H':hist_H, 'S':hist_S, 'V':hist_V}

    return histogram

def histogramHSVPlot(histogram):
    """
    plot the HSV histogram, histogram must be a dictionary with H: S: V:
    """

    hist_H = histogram['H']
    hist_S = histogram['S']
    hist_V = histogram['V']

    plt.subplot(3, 1, 1)
    plt.plot(hist_H)
    plt.xlim([0, 179])
    plt.title('Histograma H')

    plt.subplot(3, 1, 2)
    plt.plot(hist_S)
    plt.xlim([0, 255])
    plt.title('Histograma S')

    plt.subplot(3, 1, 3)
    plt.plot(hist_V)
    plt.xlim([0, 255])
    plt.title('Histograma V')

    plt.show()

    return

def createTimeDataDir(source_path):

    time_str = datetime.datetime.now().strftime('%d_%m_%y__%H_%M_%S')
    total_path = source_path + time_str
    try:
        os.makedirs(total_path)
    except OSerror as e:
        raise

    return total_path + '/'

def main():

    rospack = rospkg.RosPack()
    created_path = createTimeDataDir(rospack.get_path("ada_tutorial") +'/spoon_data/')

    rospy.init_node('spoon_perception',anonymous = True)

    inteligent_spoon = SpoonPerception('/camera/image_raw',record=True,record_dir=created_path)
    inteligent_spoon.startRetreivingImages()

    try:
        inteligent_spoon.calibrationSpoon()
        inteligent_spoon.startDetectObjectSrv()
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Donw')

    cv2.destroyAllWindows()
    plt.close()

if __name__ == '__main__':
    main()
