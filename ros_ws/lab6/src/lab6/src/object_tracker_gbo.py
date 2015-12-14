#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image

from std_msgs.msg import Int32, Int32MultiArray

import cv,cv2,time,sys
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from numpy.linalg import *

class ObjectTracker:
  def __init__(self):
    self.thresh_img = None

    # ROS to CV image bridge
    self.bridge = CvBridge()
    
    #Publishes an image after image processing
    self.pub = rospy.Publisher('processed_image', Image)

    #Publishes the image coordinates of the image
    self.pub_blue = rospy.Publisher('/zumy/blue', Int32MultiArray, queue_size=1)
    self.pub_green = rospy.Publisher('/zumy/green', Int32MultiArray, queue_size=1)
    self.pub_orange = rospy.Publisher('/zumy/orange', Int32MultiArray, queue_size=1)

    #Initialize the node
    rospy.init_node('object_tracker_gbo')

    #Subscribe to the image topic
    rospy.Subscriber("/usb_cam/image_raw", Image, self.img_received, queue_size=1)

    # Setup OpenCV windows and sliders
    def nothing(x):
        pass
    
  # Main run method for the ObjectTracker class
  def run(self):
    try:
      rospy.spin()
    except KeyboardInterrupt:
      cv2.destroyAllWindows()

  #Callback for when an image is received
  def img_received(self, message):
    # Convert ROS message to Numpy image
    self.np_image = np.array(self.bridge.imgmsg_to_cv2(message,'bgr8'))

    blur_radius = 21
    blur_sigma = 3.0

    # Blur image
    thresh_image =cv2.GaussianBlur(self.np_image, 2*(blur_radius,), blur_sigma)

    # Convert to HSV colorspace, and threshold
    thresh_image = cv2.cvtColor(thresh_image, cv2.COLOR_BGR2HSV)

    self.thresh_img = thresh_image

    for color in range(3):
      # Find red blobs in image and store it in self.thresh_img
      self.threshold_image(color)

      # Find the boxes that bound red blobs in self.thresh_img
      bounding_boxes = self.get_blob_boxes()

      # If we found red blobs, pick the largest one as the ball
      if len(bounding_boxes) > 0:
        largest_box = self.find_largest_box(bounding_boxes)
        
        # Find the centroid of the box
        centroid = np.array(largest_box).mean(0)
        if color == 0: 
          self.pub_blue.publish(data = centroid)
        elif color == 1:
          self.pub_green.publish(data = centroid)
        elif color == 2:
          self.pub_orange.publish(data = centroid)
        else:
          raise Exception('You F-ed UP!')
        
        # Convert centroid to world coordinates, and record the observation
        # z = self.compute_world_pos(centroid)
      else:
        largest_box = None
        centroid = None

      self.thresh_img = thresh_image

    cv2.waitKey(1)

  # This function take RGB image. Then blur and convert it into HSV for easy 
  # colour detection and threshold it with red part as white and all other
  # regions as black.Then return that image  
  def threshold_image(self, color):
    if color == 0: 
      COLOR_MIN = (95,110,160) # BLUE
      COLOR_MAX = (120,150,255)
    elif color == 1:
      COLOR_MIN = (80,130,131) # GREEN
      COLOR_MAX = (100,169,255) 
    elif color == 2:
      COLOR_MIN = (0,68,215)   # ORANGE
      COLOR_MAX = (61,228,255)
    else:
      raise Exception('You F-ed Up!')

    self.thresh_img = cv2.inRange(self.thresh_img, COLOR_MIN, COLOR_MAX)

  # Return boxes bounding each colored blob in the thresholded image
  def get_blob_boxes(self):
    contours,_ = cv2.findContours(self.thresh_img, 
      cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    bounding_boxes = []
    for contour in contours:
      bound_rect = cv2.boundingRect(contour)
      x_y = np.array(bound_rect[0:2])
      w_h = np.array(bound_rect[2:])
      bounding_boxes.append([list(x_y),list(x_y + w_h)])

    return bounding_boxes
    

  # Find the box with the largest area in bounding_boxes[box][point][xy]
  # Thus bounding_boxes[0] is a list of two points in the first box, 
  # bounding_box[0][1] is the second point in the first box,
  # and bounding_box[0][1][0] is the x coordinate of the above point.
  # The two points are the upper left and lower right corners of the box
  def find_largest_box(self, bounding_boxes):

#### Edit These Lines ##########################################################

    largest_dist = 0
    largest_box = bounding_boxes[0]
    for box in bounding_boxes:
      if (box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2 > largest_dist:
        largest_box = box

################################################################################
      
    return largest_box

if __name__ == '__main__':
  node = ObjectTracker()
  node.run()

