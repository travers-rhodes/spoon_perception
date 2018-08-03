#!/usr/bin/env python
import rospy
import cv2
import numpy as np

import spoon_camera_interface as sci

from spoon_perception.srv import ObjectSpoon, ObjectSpoonResponse


class SpoonObjectDetector:
  """class that detects an object on the spoon
  """

  def __init__(self, spoon_camera, number_frames=5):
    """

    """
    self.spoon_camera = spoon_camera

    self.spoon_histogram = None #this will be a dict of H S V histograms
    self.calibrated = False #flag to know if the detection is calibrated
    self.detect_object_srv = None

    self.number_frames = number_frames # number of frames that it takes to make the decision

    self.spoon_histogram = self.getEmptySpoonHistogram()
    return

  def startDetectObjectSrv(self):
    self.detect_object_srv = rospy.Service('detect_object_spoon',ObjectSpoon,self.detectObject)
    return
  
  def detectObject(self,req):
    distances = []
    for i in range(self.number_frames):
        image = self.spoon_camera.getImage() 
        histogram = getHSVHistogram(image, self.spoon_camera.spoon_mask)
        d = cv2.compareHist(self.spoon_histogram['H'], histogram['H'],
                            cv2.cv.CV_COMP_CORREL)
        distances.append(d)

    image_dir = ""

    return ObjectSpoonResponse(np.mean(distances), image_dir)

  def getEmptySpoonHistogram(self):
    """this function gets the color histogram for the empty spoon
       must be called after getSpoonMask is called
    """
    # get the histogram without sheet of paper there
    image = self.spoon_camera.getSpoonImage()
    rospy.logwarn('Remove the  white sheet of paper from the background'
                            '--- Press ENTER to accept the image'
                            '--- Press ANY key to reject')
    while( not displayWait(image, 'Calibration') ):
      image = self.spoon_camera.getSpoonImage()
      rospy.logwarn('Remove the  white sheet of paper from the background'
                            '--- Press ENTER to accept the image'
                            '--- Press ANY key to reject')
    
    histogram = getHSVHistogram(image, self.spoon_camera.spoon_mask)

    return histogram

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

if __name__ == '__main__':
  rospy.init_node('spoon_object_detector',anonymous = True)

  spoon_camera = sci.SpoonCameraInterface('/camera/image_raw')
  object_detector = SpoonObjectDetector(spoon_camera)

  object_detector.startDetectObjectSrv()
  rospy.spin()