#!/usr/bin/env python
import cv2

class VisionTasks:
    
    def findGate(self, image):
        th1 = 0
        th2 = 200

        blurred = cv2.blur(image, (3, 3))
        kernel = np.ones((3, 3), np.uint8)
        erosion = cv2.erode(blurred, kernel, iterations=1)
        edges = cv2.Canny(erosion, th1, th2)

        dilation = cv2.dilate(edges, kernel, iterations=2)
        #cv2.imshow("dilation", dilation)
        contour = self.findBiggestContour(dilation)
        x, y, w, h = cv2.boundingRect(contour)
        centerX = x + (w / 2)
        centerY = y + (h / 2)
        if centerX == 0 or centerY == 0 or w == 0:
            centerX = -1
            centerY = -1
            w = -1
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(image, "X: " + format(centerX), (int(x) - 60, int(y) + 50), font, 0.5, (155, 250, 55), 2,
                cv2.LINE_AA)
        cv2.putText(image, "Y: " + format(centerY), (int(x) - 60, int(y) + 70), font, 0.5, (155, 255, 155), 2,
               cv2.LINE_AA)
        # display
        return (image, centerX, centerY, w)


    def findBiggestContour(self, mask):
       # Contours
       contoursArray = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
       # This code will execute if at least one contour was found
       if len(contoursArray) > 0:
           # Find the biggest contour
           biggestContour = max(contoursArray, key=cv2.contourArea)
           # Returns an array of points for the biggest contour found
           return biggestContour



