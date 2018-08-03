#!/usr/bin/env python
import cv2

class VisionTasks:
    
    def findGate(self, image):
        th1 = 100
        th2 = 200
        edges = cv2.dilate(image, None)
        blurred = cv2.blur(edges, (3,3))
        edges = cv2.Canny(blurred, th1, th2)
        #self.findBiggestContour(edges)
        x2, y2, w2, h2 = cv2.boundingRect(edges)
        centerX = x2 + (w2 / 4)
        centerY = y2 + (h2 / 2)
        cv2.circle(image, (int(centerX), int(centerY)), 3, (255, 0, 0), thickness=1, lineType=8, shift=0)
        cv2.putText(image, "X: " + format(centerX), (int(x2) - 60, int(y2) + 50), font, 0.5, (155, 250, 55), 2,
                cv2.LINE_AA)
        cv2.putText(image, "Y: " + format(centerY), (int(x2) - 60, int(y2) + 70), font, 0.5, (155, 255, 155), 2,
                cv2.LINE_AA)
        #display
        return (image, centerX, centerY, w2)


    def findBiggestContour(self, mask):
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



