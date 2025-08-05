import sys
import time
import ros_robot_controller_sdk as rrc
from readchar import readkey, key
import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(1)

board = rrc.Board()

pw_BLDC = 1650
pw_BLDC_stop = 1500
pw_Servo = 1390  # Convert angle to pulse width

board.pwm_servo_set_position(0.1, [[4, pw_BLDC]])
board.pwm_servo_set_position(0.1, [[2, pw_Servo]])

min_servo = 1300
max_servo = 1720

min_bldc = 1300
max_bldc = 1700

# Black colour range in HSV
lower_black = np.array([0, 0, 0])
upper_black = np.array([90, 60, 60])

#define two ROIs
roiLeft = (40, 170, 60, 80)
roiRight = (540, 170, 60, 80)

while True:
    frame = picam2.capture_array()
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    found_left = False
    found_right = False
    
    cv2.rectangle(frame, (roiLeft[0], roiLeft[1]), (roiLeft[0]+roiLeft[2], roiLeft[1]+roiLeft[3]), (0, 255, 255), 2)
    cv2.rectangle(frame, (roiRight[0], roiRight[1]), (roiRight[0]+roiRight[2], roiRight[1]+roiRight[3]), (0, 255, 255), 2)    
    
    for idx, (x, y, w, h) in zip(["Left", "Right"], [roiLeft, roiRight]):
        roi = frame[y:y+h, x:x+w]
        mask = cv2.inRange(roi, lower_black, upper_black)
        full_mask = cv2.inRange(frame, lower_black, upper_black)
    
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original frame
        for cnt in contours:
            if cv2.contourArea(cnt) > 100:  # filter small noise
                offset_cnt = cnt + [x, y]
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                if idx == "Left":
                    found_left = True
                elif idx == "Right":
                    found_right = True
    
    if found_left and found_right:
        print("forward")
        board.pwm_servo_set_position(0.1, [[4, pw_Servo]])
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
    elif found_right and not found_left:
        print("turn left")
        board.pwm_servo_set_position(0.1, [[4, min_servo]])
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
    elif found_left and not found_right:
        print("turn right")
        board.pwm_servo_set_position(0.1, [[4, max_servo]])
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
    else:
        print("stop")
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC_stop]])
        
    # Show the result
    cv2.imshow("Contours", frame)
    cv2.imshow("Full Mask", full_mask)

    # Break loop with 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC_stop]])
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
