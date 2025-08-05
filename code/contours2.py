import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

img = picam2.capture_array()

points = [(115,200), (525,200), (640,370), (0,370)] #ROI
imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", imgGray)
#thresholding
ret, imgThresh = cv2.threshold(imgGray, 110, 255,
cv2.THRESH_BINARY_INV)
cv2.imshow("threshold", imgThresh)
contours, hierarchy = cv2.findContours(imgThresh,
cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print("Number of Contours found = " + str(len(contours)))
for i in range(len(contours)):
    cnt = contours[i]
    area = cv2.contourArea(cnt)
    print(area)
    if(area > 100):
        cv2.drawContours(img, contours, i, (0, 255, 0), 2)
        approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
        x,y,w,h=cv2.boundingRect(approx)
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
        
cv2.imshow("contours", img)
cv2.waitKey(0)
cv2.destroyAllWindows()