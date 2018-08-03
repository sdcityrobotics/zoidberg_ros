#!/usr/bin/env python
import cv2

class VisionTasks:
    
    def findGate(self, image):
        th1 = 100
        th2 = 200
	#mask = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        edges = cv2.Canny(image, th1, th2)
        #edges = cv2.dilate(edges, None)
        self.findBiggestContour(edges)
	#self.findBiggestContour(mask)
        x2, y2, w2, h2 = cv2.boundingRect(edges)
        centerX = x2 + (w2 / 2)
        centerY = y2 + (h2 / 2)
        return (edges, centerX, centerY)


    def findBiggestContour(self, mask):
        # Contours
        contoursArray = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(contoursArray) > 0:
            # Find the biggest contour
            biggestContour = max(contoursArray, key=cv2.contourArea)
            # Returns an array of points for the biggest contour found
            return biggestContour


    def findContours(self, mask):
        # image, cnts, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2]
        cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if len(cnts) > 0:
            sortedContours = sorted(cnts, key=cv2.contourArea)
            biggest2Contours = sortedContours[-4:]
            return biggest2Contours






