#!/usr/bin/env python

import os
import rospy
import cv2
from helpers import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Color code constants
RED = 1
GREEN = 2

# Tool for getting files
file_helper = FileHelper()

training, count_train = file_helper.get_files('%s/../images/'%os.path.dirname(os.path.abspath(__file__)))

class ProcessImage:
    """
    Helper class for performing object recognition.
    """
    def __init__(self, color=GREEN):
        self.img_helper = ImageHelper()
        self.object_found = False
        self.color_found = None
        self.color_to_find = RED
        self.process_images = False
        self.sub_image = rospy.Subscriber('/realsense/color/image_raw', Image, self.image_callback)
     
    
    def save_image(self, image):
        self.last_image = image
            
    def recognize_object(self, image):
        """
        Object recognition algorithm.
        Performs a K-nearest neighbors brute force match and judges
        the object color based on the average number of matches per image class.
        """
        global training
        
        # Prevents method from running too often    
        if not self.process_images:
            return
        
        # Data structures for algorithm    
        incorrect = []
        correct = []
        
        images = {}
        images[GREEN] = [k for k in training.keys() if "green" in k]
        images[RED] = [k for k in training.keys() if "red" in k]
        matches = {}
        matches[RED] = 0
        matches[GREEN] = 0
        counter = {}
        counter[RED] = int(0)
        counter[GREEN] = int(0)
        
        print("Color to find", self.color_to_find)
        
        # Find matches and calculate averages
        for color in [RED, GREEN]:
            print("Color to check:", color)
            
            for name in images[color]:
                n_matches = len(self.get_matches(image, training[name], self.color_to_find))
                matches[color] += n_matches
                counter[color] += 1
                print(n_matches, "matches for file", name, "of color", color)
                
        avg_red = matches[RED] / counter[RED]
        avg_green = matches[GREEN] / counter[GREEN]
        
        print("avg red:", avg_red, "\navg green", avg_green)
        if avg_red > avg_green:
            self.color_found = RED
        elif avg_red < avg_green:
            self.color_found = GREEN
 
        return self.color_found == self.color_to_find
        
    def get_matches_filtered(self, img1, img2, color):
        """
        Applies color filters to images and checks for matches.
        get_matches works better, but I left this code here for demonstration
        purposes.
        """
        filtered_img1 = self.img_helper.filter(img1, color)
        filtered_img2 = self.img_helper.filter(img2, color)
        
        img1_keys, img1_descs = self.img_helper.features(filtered_img1)
        img2_keys, img2_descs = self.img_helper.features(filtered_img2)
        
        if img1_descs is None or img2_descs is None:
            return []
          
        bf = cv2.BFMatcher.create(normType=cv2.NORM_L1)
        matches = bf.knnMatch(img1_descs, trainDescriptors=img2_descs, k=2)
        good = []
        
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m])
                
        return good
        
    def get_matches(self, img1, img2, color):
        """
        Returns number of matches between the SIFT features of two images.
        Taken from the OpenCV documentation at https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html.
        """
        sift = cv2.SIFT_create()
        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)
        # BFMatcher with default params
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m])
        return good
        
    def image_callback(self, image):
        """
        Callback for ROS subscriber. Calls recognize_object.        
        """
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(image, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.save_image(cv_image) 
        self.object_found = self.recognize_object(cv_image)
        

