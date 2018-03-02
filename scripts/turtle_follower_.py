#! /usr/bin/env python
import sys
import os
import argparse
import cv2
import rospy
import roslib
from std_msgs.msg import String, Float32MultiArray
#from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class ARTagFollow:
	def __init__(self, dir_):
		self.original, self.depth = None, None
		self.i = 1 
		self.lower = np.array([100, 10, 50], dtype = "uint8") #0,48,80
		self.upper = np.array([200, 70, 100], dtype = "uint8") #20,255,255
                self.dir_ = dir_
		self.annot_file = open(self.dir_ +"annotation.txt", "w+")

		self.bench_test, self.publish_image = True, False
		rospy.init_node('turtle_follower', anonymous=True)

		self.bridge = CvBridge()
		im_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.imageCallBack, queue_size=5)
		depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depthCallBack, queue_size=5)
		#tag_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.tagPoseCallback, queue_size=5)

		self.target_pub = rospy.Publisher('target_info', String, queue_size=5)
		self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

		if self.publish_image:
			self.ProcessedRaw = rospy.Publisher('/follow/out_image', Image, queue_size=5)


		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Rospy Sping Shut down")

		cv2.namedWindow("image")
		#cv2.setMouseCallback("image", self.click_and_crop)
			
		
	# for real-time testing
	def imageCallBack(self, rgb_im):
		try:
			im_array = self.bridge.imgmsg_to_cv2(rgb_im, "bgr8")
		except CvBridgeError as e:
			print(e)
		if im_array is None:
			print ('frame dropped, skipping tracking')
		else:
			self.original = np.array(im_array)
			self.SaveData()


	# for real-time testing
	def depthCallBack(self, d_im):
		try:
			d_array = self.bridge.imgmsg_to_cv2(d_im, "32FC1")
		except CvBridgeError as e:
			print(e)
		if d_array is None:
			print ('frame dropped, skipping tracking')
		else:
			self.depth = np.array(d_array)

	# for real-time testing
	def SaveData(self):
		if self.original is not None and self.depth is not None:
			self.showFrame(self.original, 'image')
			
			cv2.imshow("image", self.original)
	                key = cv2.waitKey(1) & 0xFF
            
			
            		# if the 'r' key is pressed, reset the cropping region
            		if key == ord("f"):
                		base_cmd = Twist()
				base_cmd.linear.x = 0.2
				self.cmd_pub.publish(base_cmd)
				annotn_txt = str(self.i)+str('.jpg 0.0 0.2 \n')
				self.annot_file.write(annotn_txt)
				cv2.imwrite(self.dir_+str('images/')+str(self.i)+'.jpg', self.original)
				self.i += 1

			if key == ord("r"):
                		base_cmd = Twist()
				base_cmd.angular.z = 0.4
				self.cmd_pub.publish(base_cmd)
				annotn_txt = str(self.i)+str('.jpg 0.4 0.0 \n')
				self.annot_file.write(annotn_txt)
				cv2.imwrite(self.dir_+str('images/')+str(self.i)+'.jpg', self.original)
				self.i += 1

			if key == ord("l"):
                		base_cmd = Twist()
				base_cmd.angular.z = -0.4
				self.cmd_pub.publish(base_cmd)
				annotn_txt = str(self.i)+str('.jpg -0.4 0.0 \n')
				self.annot_file.write(annotn_txt)
				cv2.imwrite(self.dir_+str('images/')+str(self.i)+'.jpg', self.original)
				self.i += 1

			elif key == ord("q"):
				print ("Saved data")
                		self.annot_file.close()

			
				

	'''
	# for real-time testing
	def tagPoseCallback(self, msg):
		if self.original is not None and self.depth is not None:
			if msg.markers!=[]:
				self.tag_msg = msg.markers
				self.tag_pose, self.tag_orien = self.tag_msg[0].pose.pose.position, self.tag_msg[0].pose.pose.orientation  
				#print ("Found: tag", self.tag_msg[0].id)
				#print (self.tag_pose, self.tag_orien) 
				self.makemove()

			if self.bench_test:	
				self.showFrame(self.original, 'input_image')
				#self.showFrame(self.depth, 'input_depth')
		
		if self.publish_image:
			msg_frame = CvBridge().cv2_to_imgmsg(self.original, encoding="bgr8")
			self.ProcessedRaw.publish(msg_frame)
	'''
		

	def makemove(self):
		if self.tag_pose != None:
			base_cmd = Twist()
			base_cmd.linear.x = (self.tag_pose.z - 0.5)
			base_cmd.angular.z = -self.tag_pose.x*4
			self.cmd_pub.publish(base_cmd)



	##########################################################################
	###   For bench testing with dataset images ###############################
	def showFrame(self, frame, name):
		cv2.imshow(name, frame)
		cv2.waitKey(20)

	# stream images from directory Dir_
	def image_streamimg(self, Dir_):
		from eval_utils import filter_dir
		dirFiles = filter_dir(os.listdir(Dir_))
		for filename in dirFiles:
			self.original = cv2.imread(Dir_+filename)
			self.ImageProcessor()
	####################################################################################
