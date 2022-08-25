#!/usr/bin/env python

import cv2
import numpy as np
from glob import glob
from process_image import *
from sklearn.cluster import KMeans
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler

RED = 1
GREEN = 2
RED_BOUNDS = ([0, 100, 20], [10, 255, 255], [160, 100, 20], [179, 255, 255]) #(lower1, upper1, lower2, upper2)
GREEN_BOUNDS = ([30, 20, 6], [120, 255, 255])

class ImageHelper:
    """
    Helper class for filtering images and extracting SIFT features.
    """
    def __init__(self):
        self.sift_object = cv2.SIFT_create()
        
    
    def filter(self, image, color):
        """
        """
        result = image.copy()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if color == RED:
            lower1 = np.array(RED_BOUNDS[0])
            upper1 = np.array(RED_BOUNDS[1])
            lower2 = np.array(RED_BOUNDS[2])
            upper2 = np.array(RED_BOUNDS[3])
            
            lower_mask = cv2.inRange(image, lower1, upper1)
            upper_mask = cv2.inRange(image, lower2, upper2)
            
            full_mask = lower_mask + upper_mask
            
            result = cv2.bitwise_and(result, result, mask=full_mask)
        elif color == GREEN:
            lower = np.array(GREEN_BOUNDS[0])
            upper = np.array(GREEN_BOUNDS[1])
            
            mask = cv2.inRange(image, lower, upper)
            result = cv2.bitwise_and(result, result, mask=mask)
            
        return result
        
    def features(self, image):
        """
        Returns the SIFT keypoints and descriptors of the given image.
        """
        key, desc = self.sift_object.detectAndCompute(image, None)
        return (key, desc)
        
        
class FileHelper:
    """
    Helper class for collecting an image list from files in a directory.
    Much of it comes from https://kushalvyas.github.io/BOV.html, with a
    few chances to work for my program.
    """
    def __init__(self):
        pass
        
    def get_files(self, path):
        """
        Returns a dictionary of the image files from the given path.
        """
        print("File path:", path)
        imlist = {}
        count = 0
        
        """
        for each in glob(path + "*"):
            word = each.split("/")[-1]
            
            imlist[word] = []
            print(type(word))
            for imgfile in glob(path + word + "/*"):
                print("Reading file", imgfile)
                img = cv2.imread(imgfile, 0)
                imlist[word].append(img)
                count += 1
        return (imlist, count)
        """

        for imgfile in glob(path + "*"):
            print("Reading file", imgfile)
            img = cv2.imread(imgfile)
            imlist[imgfile.split("/")[-1]] = img
            count += 1
        return (imlist, count)


if __name__ == "__main__":
    helper = ImageHelper()
    img1 = cv2.imread('../images/red1.png')
    img2 = cv2.imread('../images/red5.png')
    img3 = cv2.imread('../images/secondgreen4.png')
    img4 = cv2.imread('../images/red2.png')
    
    pi = ProcessImage()
    matches = pi.get_matches(img1, img2, RED)
    mismatches = pi.get_matches(img1, img3, RED)
    
    print(len(matches), "matches \t", len(mismatches), "mismatches")
    
    pi.recognize_object(img3)
    
    print("Color found", pi.color_found)
    
    """
    filtered_img = helper.filter(img3, GREEN)
    cv2.imshow('filtered', filtered_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    filtered_img = helper.filter(img1, RED)
    filtered_img2 = helper.filter(img2, RED)
    filtered_img3 = helper.filter(img3, RED)
    
    
    bf = cv2.BFMatcher.create(normType=cv2.NORM_L1)
    img_keys, img_descs = helper.features(filtered_img)
    img2_keys, img2_descs = helper.features(filtered_img2)
    img3_keys, img3_descs = helper.features(filtered_img3)
    
    print("img3_descs:", img3_descs)
    matches = bf.match(img_descs, img3_descs)
    print("%s matches"%len(matches))
    
    print(img3_descs)
    mismatches = bf.match(img_descs, img3_descs)
    print("%s mismatches"%len(mismatches))
    
    
   
    cv2.imshow('filtered', filtered_img3)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """
    """
    features = helper.features(filtered_img)
    print(len(features[0]))
    
    file_helper = FileHelper()
    files, count = file_helper.get_files("../images/")
    print(files)
    print(type(k) for k in files.keys())
    """
 
    
