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
##### Edit These Lines  ########################################################

    # Real-world coordinates of clicked homography points
    self.world_points = [[0,0],[0,1],[1,1],[1,0]]

    # Default values of sliders. Edit this once you know what values to usef
    # for filtering
    COLOR_MIN = (170,175,0) # HSV
    COLOR_MAX = (179,255,255) #HSV
    BLUR_RADIUS = 10
    BLUR_SIGMA = 3

################################################################################

    # Homography class variables
    self.homography_state = 'Prompt'
    self.homography_image = None
    self.image_points = [] 
    self.H = None

    # Kalman filter class variables
    self.obs = []
    self.traj = [np.array([0.,0.,0.,0.])]
    self.P = np.eye(4)
    self.last_time = time.time()

    # ROS to CV image bridge
    self.bridge = CvBridge()
    
    #Publishes an image after image processing
    self.pub = rospy.Publisher('processed_image', Image)

    #Publishes the image coordinates of the image
    self.pub_centroid = rospy.Publisher('/zumy/red', Int32MultiArray, queue_size=1)

    #Initialize the node
    rospy.init_node('object_tracker_red')

    #Subscribe to the image topic
    rospy.Subscriber("/usb_cam/image_raw", Image, self.img_received, queue_size=1)
    
    # Setup OpenCV windows and sliders
    def nothing(x):
      pass

    cv2.namedWindow("Thresh", cv.CV_WINDOW_AUTOSIZE)
    cv2.createTrackbar('blur_radius', "Thresh", BLUR_RADIUS, 50, nothing)
    cv2.createTrackbar('blur_sigma', "Thresh", BLUR_SIGMA, 10, nothing)
    
    cv2.createTrackbar('H_min', "Thresh", COLOR_MIN[0], 179, nothing)
    cv2.createTrackbar('S_min', "Thresh", COLOR_MIN[1], 255, nothing)
    cv2.createTrackbar('V_min', "Thresh", COLOR_MIN[2], 255, nothing)

    cv2.createTrackbar('H_max', "Thresh", COLOR_MAX[0], 179, nothing)
    cv2.createTrackbar('S_max', "Thresh", COLOR_MAX[1], 255, nothing)
    cv2.createTrackbar('V_max', "Thresh", COLOR_MAX[2], 255, nothing)
    
    cv2.namedWindow("Trajectory vs. Observations", cv.CV_WINDOW_AUTOSIZE)
    
    cv2.namedWindow("Homography", cv.CV_WINDOW_AUTOSIZE)
    cv2.setMouseCallback("Homography", self.on_mouse_click, param=1)

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
    
    # Wait for the user to input homography points if not done already
    if self.homography_state is not 'Done':
      self.do_homography()
    else:
      # Find red blobs in image and store it in self.thresh_img
      self.threshold_image()

      # Find the boxes that bound red blobs in self.thresh_img
      bounding_boxes = self.get_blob_boxes()

      # If we found red blobs, pick the largest one as the ball
      if len(bounding_boxes) > 0:
        largest_box = self.find_largest_box(bounding_boxes)
        
        # Find the centroid of the box
        centroid = np.array(largest_box).mean(0)

        # Convert centroid to world coordinates, and record the observation
        self.pub_centroid.publish(data = centroid)
        z = self.compute_world_pos(centroid)
      else:
        largest_box = None
        centroid = None
        z = None
      
      # Draw the bounding boxes and centroid on the thresholded image output
      self.draw_boxes(bounding_boxes, largest_box, centroid)
      
      # Append the observation
      self.obs.append(z)
      
      # Run the Kalman filter to predict, and update if there is an observation
      self.kalman_filter(z)

      # Draw the observations and Kalman filtered trajectory on the comparison
      # image based on self.obs and self.traj
      self.plot_trajectory()

      # Mark the most recent filtered ball position on the output image
      self.draw_ball_pos()

      # Publish the image with the filtered ball position marked
      self.publish_output()

    # Call waitKey to render OpenCV images and get slider input
    cv2.waitKey(1)

  # Wait for user to snap picture and click 4 points
  def do_homography(self):
    if self.homography_state is 'Prompt':
      raw_input('Press Enter to Capture an Image to Compute the Homography:')
      self.capture_time = time.time() + 0.1
      self.homography_state = 'Capture'

    elif self.homography_state is 'Capture':
      if time.time() > self.capture_time:
        self.homography_image = self.np_image.copy()
        cv2.imshow("Homography", self.homography_image)
        self.homography_state = 'Points'
    
    elif len(self.image_points) == 4:
      print "Finished collecting points..."
      self.compute_homography()
      self.check_homography(6, 6, 0.2)
      self.homography_state = 'Done'
  
  # Callback function for mouse click
  def on_mouse_click(self, event, x, y, flag, param):
    if(event == cv2.EVENT_LBUTTONUP):
      print "Point Captured: (%s,%s)" % (x,y)
      self.image_points = self.image_points + [[x,y]]

  # Compute homography from self.world_points and user-clicked self.image_points
  def compute_homography(self):
    def A_rows(x,y,u,v):
      return np.array([
        [x, y, 1, 0, 0, 0, -u*x, -u*y],
        [0, 0, 0, x, y, 1, -v*x, -v*y]])
    coords = zip(self.world_points, self.image_points)
    A = np.vstack([A_rows(xy[0],xy[1],uv[0],uv[1]) for xy,uv in coords])
    b = np.array(self.image_points).reshape(8,1)
    self.H = np.vstack([inv(A).dot(b), 1.0]).reshape((3,3))
 
  # Convert world x,y to image u,v coordinates
  # x_y is a 2x1 NumPy array with the x and y coordinates
  # returns 2x1 NumPy integer array
  def compute_image_pos(self, x_y):
    x_bar = np.hstack((x_y,1.))
    u_bar = self.H.dot(x_bar)
    u_bar = u_bar/u_bar[2]
    return u_bar[0:2].astype(int)

  # Convert image u,v to world x,y coordinates
  # u_v is a 2x1 NumPy array with the u and v coordinates
  # returns a 2x1 NumPy array
  def compute_world_pos(self, u_v):
    u_bar = np.hstack((u_v,1.))
    x_bar = inv(self.H).dot(u_bar)
    x_bar = x_bar/x_bar[2]
    return x_bar[0:2]
  
  # nx is the number of tiles in the x direction
  # ny is the number of tiles in the y direction
  # length is the length of one side of a tile
  def check_homography(self,nx,ny,length):
    for i in range(nx):
      for j in range(ny):
        x_y = np.array([i*length,j*length])
        pix_center = tuple(self.compute_image_pos(x_y))
        cv2.circle(self.homography_image, pix_center, 5, 0, -1)
    cv2.imshow("Homography", self.homography_image)

  # This function take RGB image. Then blur and convert it into HSV for easy 
  # colour detection and threshold it with red part as white and all other
  # regions as black.Then return that image  
  def threshold_image(self):
    color_min = (
      cv2.getTrackbarPos('H_min','Thresh'),
      cv2.getTrackbarPos('S_min','Thresh'),
      cv2.getTrackbarPos('V_min','Thresh'))
    
    color_max = (
      cv2.getTrackbarPos('H_max','Thresh'),
      cv2.getTrackbarPos('S_max','Thresh'),
      cv2.getTrackbarPos('V_max','Thresh'))
    
    blur_radius = cv2.getTrackbarPos('blur_radius','Thresh')*2+1
    blur_sigma = (1 + cv2.getTrackbarPos('blur_sigma','Thresh'))*1.0
   
    # Blur image
    self.thresh_img = cv2.GaussianBlur(self.np_image, 2*(blur_radius,), blur_sigma)

    # Convert to HSV colorspace, and threshold
    self.thresh_img = cv2.cvtColor(self.thresh_img, cv2.COLOR_BGR2HSV)
    self.thresh_img = cv2.inRange(self.thresh_img, color_min, color_max)

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
  
  # Draw bounding boxes in red, largest box in green, and centroid as blue circle
  def draw_boxes(self, bounding_boxes, largest_box, centroid):
    # Triplicate thresholded image to get back RGB channels
    thresh_bgr = np.empty(self.thresh_img.shape + (3,))
    thresh_bgr[:,:,0] = self.thresh_img.copy()
    thresh_bgr[:,:,1] = self.thresh_img.copy()
    thresh_bgr[:,:,2] = self.thresh_img.copy()
    
    for box in bounding_boxes:
      cv2.rectangle(thresh_bgr, tuple(box[0]), tuple(box[1]), (0,0,255), 1)
    
    if largest_box is not None:
      cv2.rectangle(thresh_bgr, tuple(largest_box[0]), tuple(largest_box[1]),
        (0,255,0), 3)
      cv2.circle(thresh_bgr, tuple(centroid.astype(int)), 5, (255,0,0), 1)

    cv2.imshow("Thresh",thresh_bgr)
    

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

  # Draw the last Kalman filtered point on the output image
  def draw_ball_pos(self):
    ball_u_v = self.compute_image_pos(self.traj[-1][[0,2]])
    cv2.circle(self.np_image, tuple(ball_u_v), 5, 0, -1)

  # Publish the output image stored in self.np_image
  def publish_output(self):
    # cv_image = cv.fromarray(self.np_image)
    ros_msg = self.bridge.cv2_to_imgmsg(self.np_image, encoding="bgr8")
    self.pub.publish(ros_msg)

  # Plots both the tracked trajectory and the observations over time
  # Only shows the previous n_hist points
  def plot_trajectory(self, n_hist = 100):
    traj_image = self.np_image.copy()

    for traj_point in self.traj[-n_hist:]:
      u_v = self.compute_image_pos(traj_point[[0,2]])
      cv2.circle(traj_image, tuple(u_v), 5, 0, -1)

    for obs_point in self.obs[-n_hist:]:
      if obs_point is not None:
        u_v = self.compute_image_pos(obs_point)
        cv2.circle(traj_image, tuple(u_v), 5, 100, -1)

    cv2.imshow("Trajectory vs. Observations", traj_image)
    
  # Runs one update of the Kalman Filter
  # self.traj - state trajectory, list of 1D numpy arrays
  # self.cov - covariance matrices, list of 1D numpy arrays
  # z is the current observation - a 1D numpy array, or None
  # if z is None than the Kalman filter simply updates position estimate 
  # according to the dynamics
  def kalman_filter(self,z):
    
    # Period Length
    current_time = time.time()
    T = current_time - self.last_time
    self.last_time = current_time

    x = self.traj[-1] # x is a 1-D numpy.array of the form x = np.array([1.,2.,3.,4.])
    
#### Edit These Lines ##########################################################

    A = np.zeros((4,4))
    C = np.zeros((2,2))
    # A = 1.0 * np.eye(4)
    # A[0,1] = T
    # A[2,3] = T
    # C[0,0] = x[2] / x[0]
    # C[1,1] = x[3] / x[1]


################################################################################

    Q = 0.1*np.eye(4)
    R = 0.1*np.eye(2)

    # Dynamics prediction step
    x_hat = A.dot(x)
    P_hat = np.einsum('ij,jk,kl', A, self.P, A.T) + Q

    # Update state estimate and covariance if there is an observation
    if z is not None:
#### Edit These Lines ##########################################################

      x_plus = x_hat 
      P_plus = P_hat

################################################################################
    else:
      x_plus = x_hat
      P_plus = P_hat
      
    self.traj.append(x_plus)
    self.P = P_plus
          
if __name__ == '__main__':
  node = ObjectTracker()
  node.run()

