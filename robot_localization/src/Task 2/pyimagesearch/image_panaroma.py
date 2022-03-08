#!/usr/bin/env python

#import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import sys
import imutils

# bridge = CvBridge()


# def image1_callback(ros_image):
#   print 'got an image'
#   global bridge
#   #convert ros_image into an opencv-compatible image
#   try:
#     cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)
#     cv2.imshow("Image window 1", cv_image)
#     cv2.waitKey(100)
#   except CvBridgeError as e:
#       print(e)

# def image2_callback(ros_image):
#   print 'got an image'
#   global bridge
#   #convert ros_image into an opencv-compatible image
#   try:
#     cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)
#     cv2.imshow("Image window 2", cv_image)
#     cv2.waitKey(100)
#   except CvBridgeError as e:
#       print(e)


  #from now on, I can work exactly like with opencv
    
# def main(args):
#   rospy.init_node('image_converter', anonymous=True)
#   #for turtlebot3 waffle
#   #image_topic="/camera/rgb/image_raw/compressed"
#   #for usb cam
#   #image_topic="/usb_cam/image_raw"
#   image1_sub = rospy.Subscriber("/usb_cam1/image_raw",Image, image1_callback)
#   image2_sub = rospy.Subscriber("/usb_cam2/image_raw",Image, image2_callback)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X and initialize the
		# cached homography matrix
		self.isv3 = imutils.is_cv3()
		self.cachedH = None

	def stitch(self, images, ratio=0.75, reprojThresh=4.0):
		# unpack the images
		(imageB, imageA) = images
		# if the cached homography matrix is None, then we need to
		# apply keypoint matching to construct it
		if self.cachedH is None:
			# detect keypoints and extract
			(kpsA, featuresA) = self.detectAndDescribe(imageA)
			(kpsB, featuresB) = self.detectAndDescribe(imageB)
			# match features between the two images
			M = self.matchKeypoints(kpsA, kpsB,
				featuresA, featuresB, ratio, reprojThresh)
			# if the match is None, then there aren't enough matched
			# keypoints to create a panorama
			if M is None:
				return None
			# cache the homography matrix
			self.cachedH = M[1]
		# apply a perspective transform to stitch the images together
		# using the cached homography matrix
		result = cv2.warpPerspective(imageA, self.cachedH,
			(imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
		result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
		# return the stitched image
		#print("result" , result)
		return result

	def detectAndDescribe(self, image):
		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# Print version string
		print "OpenCV version :  {0}".format(cv2.__version__)

		# Extract major, minor, and subminor version numbers
		(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
		print "Major version :  {0}".format(major_ver)
		print "Minor version :  {0}".format(minor_ver)
		print "Submior version :  {0}".format(subminor_ver)


		# check to see if we are using OpenCV 3.X
		if self.isv3:
			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:
			# detect keypoints in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

		# convert the keypoints from KeyPoint objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of keypoints and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
		ratio, reprojThresh):
		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:
			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:
			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)

			# return the matches along with the homograpy matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homograpy could be computed
		return None

	def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
		# initialize the output visualization image
		(hA, wA) = imageA.shape[:2]
		(hB, wB) = imageB.shape[:2]
		vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
		vis[0:hA, 0:wA] = imageA
		vis[0:hB, wA:] = imageB

		# loop over the matches
		for ((trainIdx, queryIdx), s) in zip(matches, status):
			# only process the match if the keypoint was successfully
			# matched
			if s == 1:
				# draw the match
				ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
				ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
				cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

		# return the visualization
		return vis 


# if __name__ == '__main__':
#     main(sys.argv)