#!/usr/bin/env python
"""This is a ROS node written in Python for tracking and estimating position and velocity of the
   Ar tag markers which are detected within a vision of usb camera.
   Firstly this node subscribes to ROS topic "multiple_pose_marker_2" which we have published in a  ROS
   node named "multiple_pose_marker". Output of "multiple_pose_marker"node (Pose and Orientation) are further used 
   in this ROS node to estimate the position using kalman filter.The kalman filter here are designed in such a way that 
   we can track the object using Ar tag marker and also calculate and Predict it's position and velocity even if
   marker is invisible or not detected because of any reason.
"""


import rospy  # To be imported to write a ROS Node in Python.
import tf

# To import the entire module,This enables to work with all the functions present in the module.
import numpy as np

#To import Maths related library functions 
from math import pow
import math

"""FilterPy is a Python library that implements a filters, most notably Kalman filters.
  As this node uses FilterPy library for implementing kalman filtering we need to import some of its 
  in build functions using following command"""

from filterpy.kalman import KalmanFilter

# geometry_msgs provides messages for common geometric primitives such as points, vectors, and poses.
from geometry_msgs.msg import TwistStamped # Importing Message type "TwistStamped" to estimate linear and angular velocity

# The ar_track_alvar_msgs.msg is to reuse the ar_track_alvar_msgs/AlvarMarkers message type for publishing.
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

# To use the conversion from quaternions provided by an Odometry message to Euler angles Roll, Pitch and Yaw.
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Definition and declaration of Some Global and local variables
new_values = AlvarMarker()
roll = pitch = yaw = 0.0
flag = False
flag1 = False
a = {}
b = {}
c = list()
y = {}
old_marker_number = 0
yawfinal = {}
counter = {}
old_yaw = {}
new_yaw = {}
diff = {}

def callback(req):

    global flag
    global flag1
    global a
    global b
    global old_marker_number
    global filter_values
    #global abc
    global new_values
    global y
    d = {}
    global yawfinal
    global counter
    global old_yaw
    global new_yaw
    global diff
    global counter

    filter_values = AlvarMarkers()
    #Total numbers of visible markers are calculated
    marker_number = len(req.markers)
    rospy.loginfo("Total number of markers %d detected ", marker_number)

    filter_values.header.frame_id = "Kalman_Camera"
    filter_values.header.stamp = rospy.get_rostime()

    if marker_number != 0 and old_marker_number == marker_number:
        flag = True
        old_marker_number = marker_number 
    elif marker_number == 0 or marker_number != old_marker_number:
        flag = False
        flag1 = False

    if flag == True and flag1 == False:
        for i in range(0, marker_number):
            marker_id = req.markers[i].id  
            if a.has_key(marker_id):
                pass
            else:
                a[marker_id] = ClassKalman(marker_id)
                old_yaw[marker_id] = 0
                counter[marker_id] = 0
        flag1 = True

    if flag1 == True:
        for i in range(0, marker_number):
            marker_id = req.markers[i].id
	    
# x,y and z are the Pose of detected markers 
            x = req.markers[i].pose.pose.position.x
            y = req.markers[i].pose.pose.position.y
            z = req.markers[i].pose.pose.position.z
	   
# roll,pitch and yaw are the euler angles which we have converted from quaternion angles(Orientation x,y,z and w)
            global roll, pitch, yaw
            orientation_q = req.markers[i].pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            
            if yaw < 0:
            	while yaw < 0:
            		yaw = yaw + 6.28

            new_yaw[marker_id] = yaw

            diff[marker_id] = new_yaw[marker_id] - old_yaw[marker_id]
            
            if diff[marker_id] > 1 or diff[marker_id] < -1:
            	counter[marker_id] = counter[marker_id] + 1
            
            yawfinal[marker_id] = -6.28*counter[marker_id] + yaw

# States needs to be tracked using kalman filtering are x,y and theta(yaw in this case) are stored in input_value           	
            input_value = [x, y, yawfinal[marker_id]]
            b[marker_id] = input_value
            d[marker_id] = 1
            new_flag = True
            old_yaw[marker_id] =  new_yaw[marker_id]
	         
# In this "For loop" new object is created for each of new marker
    for key, values in b.iteritems(): #iteritem: loop over all items , runs loops for all contained items
        try:
            new_values = AlvarMarker()
            a[key].printmarker()

# If marker is available in the list and 'is visible', predict and update the kalman filter states.
            if d.has_key(key):
                a[key].predictkalman(values)

# If marker is available in the list and 'is not visible'by camera, only predict the kalman filter states. 
            else:
                a[key].predictkalmannoupdate()
            filter_values.markers.append(new_values)
        except:
            UnboundLocalError

    pub.publish(filter_values)
    old_marker_number = marker_number

    #print("time_end =" , rospy.get_rostime().to_nsec())

""" This is a model of kalman filter writeen using FilterPy python library.
    Here different matrices are defined which needs to be included in a designing of kalman filter.
    This is "constant velocity model" where e assume that the velocity of moving object 
    would be constant.
"""
# Define function/functions to provide the required functionality
class ClassKalman:
    
    global filter_values
    global new_values

    def __init__(self, marker_id):
        self.marker_id = marker_id 
        self.get_kalman()

    def printmarker(self):
        print "Detected marker ID = %d" % self.marker_id
        new_values.id = self.marker_id  

    def get_kalman(self):

        self.f = KalmanFilter(dim_x=6, dim_z=3) #self.f is object and kalman filter type

        # x = filter state matrix (6*1)
        self.f.x = np.array([[0.2],  # position_x
                        	[0.1],  # position_y
                        	[0.2],  # position_yaw
                        	[0.],  # velocity_x
                        	[0.],  # velocity_y
                       		[0.]])  # velocity_yaw

        # F = state transition matrix
        # what about dt in matrix F (6*6)
        self.f.F = np.array([[1., 0., 0., 1.0, 0., 0.],
                        	[0., 1., 0., 0., 1.0, 0.],
                        	[0., 0., 1., 0., 0., 1.0],
                       		[0., 0., 0., 1., 0., 0.],
                        	[0., 0., 0., 0., 1., 0.],
                        	[0., 0., 0., 0., 0., 1.]])

        # H = measurement function (3*6)
        self.f.H = np.array([[1., 0., 0., 0., 0., 0.],
                        	 [0., 1., 0., 0., 0., 0.],
                        	 [0., 0., 1., 0., 0., 0.]])

        # P = covariance matrix. Here I take advantage of the fact that P already contains np.eye(dim_x), and just multiply by the uncertainty:
        self.f.P *= 1000

        # R = measurement noise
        self.f.R = np.array([[0.05, 0., 0.],
                             [0., 0.05, 0.],
                             [0., 0., 0.05]])

# Predict and update function
    def predictkalman(self, input_value):
        self.f.predict()
        self.f.update(input_value)
	print("Estimated Position_x3 ="           + str(self.f.x[0]))
	print("Estimated Position_y3 ="           + str(self.f.x[1]))
	print("Estimated Orientation_yaw3 ="      + str(self.f.x[2]))
	print("Estimated Velocity_x3 ="           + str(self.f.x[3]))
	print("Estimated Velocity_y3 ="           + str(self.f.x[4]))
	print("Estimated Angular_Velocity_yaw3 =" + str(self.f.x[5]))
	print("\n")
        new_values.pose.pose.position.x = (self.f.x[0])          # Estimated Position x 
        new_values.pose.pose.position.y = (self.f.x[1])	         # Estimated Position y 
        new_values.pose.pose.orientation.z = (self.f.x[2])       # Estimated Orientation yaw
    	new_values.pose.pose.orientation.x = (self.f.x[3])       # Estimated Velocity x
	new_values.pose.pose.orientation.y = (self.f.x[4])       # Estimated Velocity y
        new_values.pose.pose.orientation.w = (self.f.x[5]*6.28)  # Estimated Angular_Velocity yaw

# Predict only function
    def predictkalmannoupdate(self):
        self.f.predict()
        print(self.f.x)
	print("Predicted Position_x3 ="           + str(self.f.x[0]))
	print("Predicted Position_y3 ="           + str(self.f.x[1]))
	print("Predicted Orientation_yaw3 ="      + str(self.f.x[2]))
	print("Predicted Velocity_x3 ="           + str(self.f.x[3]))
	print("Predicted Velocity_y3 ="           + str(self.f.x[4]))
	print("Predicted Angular_Velocity_yaw3 =" + str(self.f.x[5]))
	print("\n")
        new_values.pose.pose.position.x = (self.f.x[0])          # Estimated Position x 
        new_values.pose.pose.position.y = (self.f.x[1])	         # Estimated Position y 
        new_values.pose.pose.orientation.z = (self.f.x[2])       # Estimated Orientation yaw
    	new_values.pose.pose.orientation.x = (self.f.x[3])       # Estimated Velocity x
	new_values.pose.pose.orientation.y = (self.f.x[4])       # Estimated Velocity y
        new_values.pose.pose.orientation.w = (self.f.x[5]*6.28)  # Estimated Angular_Velocity yaw

if __name__ == '__main__':
# Add here the name of the ROS. In ROS, names are unique named.
# This is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master
    rospy.init_node('object_tracking_KF_2')

# Subscribes to topic name "ar_pose_marker_1" using the message type "AlvarMarkers".
# The callback argument calls the function which is defined earlier
    sub = rospy.Subscriber('multiple_pose_marker_2', AlvarMarkers, callback)

# Declaration of node publishing to the "filter_values_2" topic using the message type "AlvarMarkers"
# The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
    pub = rospy.Publisher('filter_values_2', AlvarMarkers, queue_size=10)

# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()







