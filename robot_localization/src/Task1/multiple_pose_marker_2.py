#!/usr/bin/env python 
#Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script

""" This is a ROS node written in Python which is
    subscribing to ROS topic "ar_pose_marker" of ROS package
    (ar_track_alvar/AlvarMarkers).
    "ar_track_alvar" is a ROS wrapper for Alvar, an open source AR tag tracking library.
    It servers a purpose of Identifying and tracking the pose of individual AR tags
"""


import rospy  #To be imported to write a ROS Node in Python.
import tf 

#To import the entire module,This enables to work with all the functions present in the module.
import numpy as np
#To import Maths related library functions 
from math import pow
import math

#The ar_track_alvar_msgs.msg is to reuse the ar_track_alvar_msgs/AlvarMarkers message type for publishing.
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker 
#To use the conversion from quaternions provided by an Odometry message to Euler angles Roll, Pitch and Yaw.
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#The output of ar_pose_marker is a Pose and Orientation of detected Markers.Declaring these variables as a Python dictionary
x = {}
y = {}
z = {}
orientation_x = {}
orientation_y = {}
orientation_z = {}
orientation_w = {}
roll = {}
pitch = {}
yaw = {}
a = {}

# Defining a Python function with a argument req.
def callback(req):

    # Defining and declaring Global variables outside a function and them inside a function.
    global a,x,y,z,orientation_w,orientation_x,orientation_y,orientation_z,roll,pitch,yaw

    # Total numbers of visible markers calculated
    marker_number = len(req.markers)
    rospy.loginfo("Total number of markers detected = %d", marker_number)
        
    filter_values = AlvarMarkers()

    filter_values.header.frame_id = "Kalman_Camera"
    filter_values.header.stamp = rospy.get_rostime()

# Calculates Pose and orientation of an all individual Ar tags according to their marker_id
    if marker_number > 0:
        print("mpm1 stamp right img stamp =", req.markers[-1].header.stamp,"time_end =" , rospy.get_rostime().to_nsec())

# Calculates Pose and orientation of an all individual Ar tags according to their marker_id
    for i in range(0, marker_number):

        marker_id = req.markers[i].id
        rospy.loginfo("Detected marker ID in right image = %d", marker_id)

        a[marker_id] = marker_id #Stores all the marker_id's in a Dictionary a{}

        x[marker_id] = req.markers[i].pose.pose.position.x
	rospy.loginfo("x_Position = %f meters", x[marker_id])
        x[marker_id] = req.markers[i].pose.pose.position.x + 0.411581 + 0.0522298
	rospy.loginfo("x_Position = %f meters", x[marker_id])
        y[marker_id] = req.markers[i].pose.pose.position.y
	rospy.loginfo("y_Position = %f meters", y[marker_id])
        z[marker_id] = req.markers[i].pose.pose.position.z
	rospy.loginfo("z_Position = %f meters", z[marker_id])

        orientation_x[marker_id] = req.markers[i].pose.pose.orientation.x
        orientation_y[marker_id] = req.markers[i].pose.pose.orientation.y
        orientation_z[marker_id] = req.markers[i].pose.pose.orientation.z
        orientation_w[marker_id] = req.markers[i].pose.pose.orientation.w
      
        orientation_q = req.markers[i].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll[marker_id], pitch[marker_id], yaw[marker_id]) = euler_from_quaternion(orientation_list)

	rospy.loginfo("Orientation Roll= %f radians", roll[marker_id])
        rospy.loginfo("Orientation Pitch = %f radians", pitch[marker_id])
        rospy.loginfo("Orientation Yaw = %f radians\n", yaw[marker_id])

# Publishes the ar_pose_marker values 
    for key, values in a.iteritems():
        new_values = AlvarMarker()          
        new_values.header.frame_id = "Kalman_Camera"
        new_values.id = a[key]
        new_values.header.stamp = rospy.get_rostime()
    
        new_values.pose.pose.position.x = x[key]
        new_values.pose.pose.position.y = y[key]
        new_values.pose.pose.position.z = z[key]
        new_values.pose.pose.orientation.x = orientation_x[key]
        new_values.pose.pose.orientation.y = orientation_y[key]
        new_values.pose.pose.orientation.z = orientation_z[key]
        new_values.pose.pose.orientation.w = orientation_w[key]
            
        filter_values.markers.append(new_values)

    pub.publish(filter_values) #Publishes the values 


if __name__ == '__main__':
# Add here the name of the ROS. In ROS, names are unique named.
# Defines the name of node.Until rospy has this information, it cannot start communicating with the ROS Master
    rospy.init_node('multiple_pose_marker_2', anonymous=True)

# Subscribes to topic name "ar_pose_marker_3" using the message type "AlvarMarkers".
# The callback argument calls the function which is defined earlier
    sub = rospy.Subscriber('ar_pose_marker_3', AlvarMarkers, callback)

# Declaration of node publishing to the "multiple_pose_marker_2" topic using the message type "AlvarMarkers"
# The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
    pub = rospy.Publisher('multiple_pose_marker_2', AlvarMarkers, queue_size=10)

# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
