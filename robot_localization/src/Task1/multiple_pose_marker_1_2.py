#!/usr/bin/env python 
#Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script



""" This is a ROS node written in Python which is
    subscribing to ROS topic "ar_pose_marker_1" and "ar_pose_marker_3" of ROS package
    (ar_track_alvar/AlvarMarkers) and publishing the topic multiple_pose_marker_1 and 'multiple_pose_marker_2' 
    under the message type AlvarMarkers
    "ar_track_alvar" is a ROS wrapper for Alvar, an open source AR tag tracking library.
    It servers a purpose of Identifying and tracking the pose of individual AR tags with unique marker ID.
"""

import rospy  #To be imported to write a ROS Node in Python.
import tf 
from copy import deepcopy

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
x1 = {}
y1 = {}
z1 = {}
orientation_x1 = {}
orientation_y1 = {}
orientation_z1 = {}
orientation_w1 = {}
roll1 = {}
pitch1 = {}
yaw1 = {}
a1 = {}

x2 = {}
y2 = {}
z2 = {}
orientation_x2 = {}
orientation_y2 = {}
orientation_z2 = {}
orientation_w2 = {}
roll2 = {}
pitch2 = {}
yaw2 = {}
a2 = {}

left_markers_len = 0
leftmarkers_copy = {}

# Defining a Python function for checking the duplicates in the markers detected by the right camera.
def checkduplicate(right_marker_id, rx, ry, rz):
    global left_markers_len, leftmarkers_copy

    leftmarkers = deepcopy(leftmarkers_copy)
    len = left_markers_len

    for i in range(0, len):
        if right_marker_id != leftmarkers[i].id:
           continue
        
        lx = leftmarkers[i].pose.pose.position.x
        ly = leftmarkers[i].pose.pose.position.y
        lz = leftmarkers[i].pose.pose.position.z
        d = math.sqrt((lx - rx) ** 2 + (ly - ry) ** 2)
        rospy.loginfo("Calculated d = %f", d)
        if d < 0.072:
           rospy.loginfo("Duplicate detected for marker Id = %d", right_marker_id)
           return True

    rospy.loginfo("No Duplicates detected")
    return False



# Defining a Python function with a argument req.
def callback1(req):

    # Defining and declaring Global variables outside a function and them inside a function.
    global a1,x1,y1,z1,orientation_w1,orientation_x1,orientation_y1,orientation_z1,roll1,pitch1,yaw1, left_markers_len, leftmarkers_copy

    # Total numbers of visible markers calculated
    marker_number = len(req.markers)
    left_markers_len = marker_number
    leftmarkers_copy = deepcopy(req.markers)
    
    rospy.loginfo("Total number of markers detected in left image = %d", marker_number)
        
    filter_values = AlvarMarkers()

    filter_values.header.frame_id = "Kalman_Camera"
    filter_values.header.stamp = rospy.get_rostime()

    if marker_number > 0:
        print("mpm1 stamp left img stamp =", req.markers[-1].header.stamp,"time_end =" , rospy.get_rostime().to_nsec()) 
    


# Calculates Pose and orientation of an all individual Ar tags according to their marker_id
    for i in range(0, marker_number):

        marker_id = req.markers[i].id
        rospy.loginfo("Detected marker ID in left image = %d", marker_id)


        a1[marker_id] = marker_id #Stores all the marker_id's in a Dictionary a{}

        x1[marker_id] = req.markers[i].pose.pose.position.x
	rospy.loginfo("x_Position = %f meters", x1[marker_id])
        y1[marker_id] = req.markers[i].pose.pose.position.y
	rospy.loginfo("y_Position = %f meters", y1[marker_id])
        z1[marker_id] = req.markers[i].pose.pose.position.z
	rospy.loginfo("z_Position = %f meters", z1[marker_id])

        orientation_x1[marker_id] = req.markers[i].pose.pose.orientation.x
        orientation_y1[marker_id] = req.markers[i].pose.pose.orientation.y
        orientation_z1[marker_id] = req.markers[i].pose.pose.orientation.z
        orientation_w1[marker_id] = req.markers[i].pose.pose.orientation.w
      
        orientation_q = req.markers[i].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll1[marker_id], pitch1[marker_id], yaw1[marker_id]) = euler_from_quaternion(orientation_list)

	rospy.loginfo("Orientation Roll= %f radians", roll1[marker_id])
        rospy.loginfo("Orientation Pitch = %f radians", pitch1[marker_id])
        rospy.loginfo("Orientation Yaw = %f radians\n", yaw1[marker_id])

# Publishes the ar_pose_marker values 
    for key, values in a1.iteritems():
        new_values = AlvarMarker()          
        new_values.header.frame_id = "Kalman_Camera"
        new_values.id = a1[key]
        new_values.header.stamp = rospy.get_rostime()
    
        new_values.pose.pose.position.x = x1[key]
        new_values.pose.pose.position.y = y1[key]
        new_values.pose.pose.position.z = z1[key]
        new_values.pose.pose.orientation.x = orientation_x1[key]
        new_values.pose.pose.orientation.y = orientation_y1[key]
        new_values.pose.pose.orientation.z = orientation_z1[key]
        new_values.pose.pose.orientation.w = orientation_w1[key]
            
        filter_values.markers.append(new_values)

    pub1.publish(filter_values) #Publishes the values 

# Defining a Python function with a argument req.
def callback2(req):

    # Defining and declaring Global variables outside a function and them inside a function.
    global a2,x2,y2,z2,orientation_w2,orientation_x2,orientation_y2,orientation_z2,roll2,pitch2,yaw2

    # Total numbers of visible markers calculated
    marker_number = len(req.markers)
    rospy.loginfo("Total number of markers detected in right image= %d", marker_number)
        
    filter_values = AlvarMarkers()

    filter_values.header.frame_id = "Kalman_Camera"
    filter_values.header.stamp = rospy.get_rostime()


# Calculates Pose and orientation of an all individual Ar tags according to their marker_id
    for i in range(0, marker_number):

        marker_id = req.markers[i].id
        rospy.loginfo("Detected marker ID in right image= %d", marker_id)

        if checkduplicate(marker_id, req.markers[i].pose.pose.position.x + 0.4638108, req.markers[i].pose.pose.position.y, req.markers[i].pose.pose.position.z):
            continue

        a2[marker_id] = marker_id #Stores all the marker_id's in a Dictionary a{}

        x2[marker_id] = req.markers[i].pose.pose.position.x
	rospy.loginfo("x_Position = %f meters", x2[marker_id])
        x2[marker_id] = req.markers[i].pose.pose.position.x + 0.411581 + 0.0522298
	rospy.loginfo("x_Position = %f meters", x2[marker_id])
        y2[marker_id] = req.markers[i].pose.pose.position.y
	rospy.loginfo("y_Position = %f meters", y2[marker_id])
        z2[marker_id] = req.markers[i].pose.pose.position.z
	rospy.loginfo("z_Position = %f meters", z2[marker_id])

        
        orientation_x2[marker_id] = req.markers[i].pose.pose.orientation.x
        orientation_y2[marker_id] = req.markers[i].pose.pose.orientation.y
        orientation_z2[marker_id] = req.markers[i].pose.pose.orientation.z
        orientation_w2[marker_id] = req.markers[i].pose.pose.orientation.w
      
        orientation_q = req.markers[i].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll2[marker_id], pitch2[marker_id], yaw2[marker_id]) = euler_from_quaternion(orientation_list)

	rospy.loginfo("Orientation Roll= %f radians", roll2[marker_id])
        rospy.loginfo("Orientation Pitch = %f radians", pitch2[marker_id])
        rospy.loginfo("Orientation Yaw = %f radians\n", yaw2[marker_id])

#Prints the image frame id and the rostime for calculating processing time
    if marker_number > 0:
        print("mpm1 stamp right img stamp =", req.markers[-1].header.stamp,"time_end =" , rospy.get_rostime().to_nsec()) 

# Publishes the ar_pose_marker values 
    for key, values in a2.iteritems():
        new_values = AlvarMarker()          
        new_values.header.frame_id = "Kalman_Camera"
        new_values.id = a2[key]
        new_values.header.stamp = rospy.get_rostime()
    
        new_values.pose.pose.position.x = x2[key]
        new_values.pose.pose.position.y = y2[key]
        new_values.pose.pose.position.z = z2[key]
        new_values.pose.pose.orientation.x = orientation_x2[key]
        new_values.pose.pose.orientation.y = orientation_y2[key]
        new_values.pose.pose.orientation.z = orientation_z2[key]
        new_values.pose.pose.orientation.w = orientation_w2[key]
            
        filter_values.markers.append(new_values)

    pub2.publish(filter_values) #Publishes the values 

if __name__ == '__main__':
# Add here the name of the ROS. In ROS, names are unique named.
# Defines the name of node.Until rospy has this information, it cannot start communicating with the ROS Master
    rospy.init_node('multiple_pose_marker_1', anonymous=True)

# Subscribes to topic name "ar_pose_marker_1" and "ar_pose_marker_3" using the message type "AlvarMarkers".
# The callback argument calls the function which is defined earlier
    sub1 = rospy.Subscriber('ar_pose_marker_1', AlvarMarkers, callback1)
    sub2 = rospy.Subscriber('ar_pose_marker_3', AlvarMarkers, callback2)

# Declaration of node publishing to the "multiple_pose_marker_1" and "multiple_pose_marker_3" topic using the message type "AlvarMarkers"
# The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
    pub1 = rospy.Publisher('multiple_pose_marker_1', AlvarMarkers, queue_size=10)
    pub2 = rospy.Publisher('multiple_pose_marker_2', AlvarMarkers, queue_size=10)

# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
