import time
from picamera2 import Picamera2, Preview
import cv2

picam = Picamera2()
picam.preview_configuration.main.size = (640,480)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.controls.FrameRate = 100
picam.preview_configuration.align()
picam.configure("preview")
picam.start()
time.sleep(2)
picam.capture_file("test.jpg")
picam.close()
img = cv2.imread("test.jpg", cv2.IMREAD_COLOR)
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()