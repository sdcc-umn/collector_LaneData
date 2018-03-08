import sys
import numpy as np
import cv2
import os


'''
Resizing all images in --src folder and saves in --dest folder 
------- speficy image size in --w and --h
'''

class ProcessImg(object):
	def __init__(self):
		self.im_ext_ = ['jpg', 'jpeg', 'bmp', 'png', 'tiff','ppm', 'pgm'] 
		self.lower = np.array([0, 0, 0], dtype = "uint8")
		self.upper = np.array([100, 200, 100], dtype = "uint8")

	def check_file_ext(self, f_name):
		    for ext_ in self.im_ext_:
			if f_name.lower().endswith(ext_):
			    return True
		    return False


	def process_all(self, src, dest):
		dirFiles = os.listdir(src)
		for im_file in dirFiles:
			if self.check_file_ext(im_file):
				im = cv2.imread(src+im_file)
				hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV) #Convert to HSV color space 
				mask2 = cv2.inRange(hsv, self.lower, self.upper) # Create a binary thresholded image
				_, thresholded = cv2.threshold(mask2, 20, 255, 0)
				cv2.imwrite(dest+im_file, thresholded)


src_folder = '/home/irvlab/code/py_work/hough/src/'
dst_folder = '/home/irvlab/code/py_work/hough/dest/'
pc = ProcessImg()
pc.process_all(src_folder, dst_folder)
