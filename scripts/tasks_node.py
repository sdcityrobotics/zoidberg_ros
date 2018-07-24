#!/usr/bin/env python
import cv2
import numpy as np

class Tasks:
    
    def __init__(self, image):
        self.image = image

        ## RED ##
        # lower mask (0-10)
        self.low_red1 = np.array([0,50,50])
        self.hi_red1 = np.array([10,255,255])
        # upper mask (170-180)
        self.low_red2 = np.array([170,50,50])
        self.hi_red2 = np.array([180,255,255])

        ## ORANGE ##
        # lower mask (0-10)
        self.low_orange = np.array([5,50,50])
        self.hi_orange = np.array([25,255,255])


    def findObject(self, object):
        
        if object == "qualGate":
            #mask = findRed(self.image, self.low_red1, self.hi_red1, self.low_red2, self.hi_red2)
            mask = findColor(self.image, self.low_orange, self.hi_orange)

            #contour = findBiggestContour(mask)
            contours = findBiggestTwoContours(mask)

        c, x, y = getCoords(contours)
        return (c, x, y)


    def findRed(self, mask1low, mask1hi, mask2low, mask2hi):
        img_yuv = cv2.cvtColor(self.image, cv2.COLOR_BGR2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])

        # convert the YUV image back to RGB then HSV format
        img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask0 = cv2.inRange(img, mask1low, mask1hi)
        mask1 = cv2.inRange(img, mask2low, mask2hi)
        # join my masks
        mask = mask0+mask1
        return mask


    def findColor(self, masklow, maskhi):
        img_yuv = cv2.cvtColor(self.image, cv2.COLOR_BGR2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])

        # convert the YUV image back to RGB then HSV format
        img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(img, masklow, maskhi)
        return mask


    def findBiggestContour(mask):
        # Contours
        # READ MORE: https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html
        # READ MORE: https://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a

        # Create an array of contour points
        # READ MORE: https://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
        contoursArray = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)[-2]

        # This code will execute if at least one contour was found
        if len(contoursArray) > 0:
            # Find the biggest contour
            biggestContour = max(contoursArray, key=cv2.contourArea)
            # Returns an array of points for the biggest contour found
            return biggestContour


    def findBiggestTwoContours(mask):
        image, cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = sorted(cnts, key=cv2.contourArea)
        biggestContours = cnt[-2:]
        return biggestContours


    def findShape(c):
        shape = ""
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"

        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"

        # return the name of the shape
        return shape


    def getCoords(self,contours):
        cX = 0
        cY = 0
        frame = self.image

        for c in contours:

            shape = findShape(c)

            if(shape == "square" or shape == "rectangle"):

                x, y, w, h = cv2.boundingRect(c)

                centerX = x+(w/2)
                centerY = y+(h/2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Choose the font for the text we will display
                font = cv2.FONT_HERSHEY_SIMPLEX

                # Put the coordinates of contour on the screen and have them move with the object
                cv2.putText(frame, "X: " + format(x), (int(x) - 60, int(y) + 50), font, 0.6, (155, 250, 55), 2, cv2.LINE_AA)
                cv2.putText(frame, "Y: " + format(y), (int(x) - 60, int(y) + 70), font, 0.6, (155, 255, 155), 2, cv2.LINE_AA)
                cv2.putText(frame, "W: " + format(w), (int(x) - 60, int(y) + 90), font, 0.6, (215, 250, 55), 2, cv2.LINE_AA)
                cv2.putText(frame, "H: " + format(h), (int(x) - 60, int(y) + 110), font, 0.6, (155, 250, 155), 2, cv2.LINE_AA)

                cX = cX + centerX
                cY = cY + centerY

        if len(contours) == 2:
            cX = cX/2
            cY = cY/2

        return len(contours), cX, cY
