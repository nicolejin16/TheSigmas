import sys
import time
import threading
import subprocess
import cv2
import numpy as np
from picamera2 import Picamera2
import ros_robot_controller_sdk as rrc
import os
import signal

# Handle debug mode
n = len(sys.argv)
debugMode = 0
if n > 1 and sys.argv[1] == "Debug":
    debugMode = 1

button_pressed = False

start = True

# === Button Listening Function ===
def listen_to_button_events():
    global button_pressed
    command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
    process = subprocess.Popen(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    button_id = None
    while not button_pressed:
        output = process.stdout.readline()
        if output:
            line = output.strip()
            if line.startswith("id:"):
                button_id = int(line.split(":")[1].strip())
            elif line.startswith("state:"):
                state = int(line.split(":")[1].strip())
                if button_id == 1 and state == 1:
                    print("Button 1 pressed: Starting robot.")
                    button_pressed = True
                    break

def Stop(signum, frame):
    global start
    start = False
    print('closing...')


board = rrc.Board()

board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(1)

def check_node_status():
    command = 'source /home/ubuntu/.zshrc && ros2 topic list'
    result = subprocess.run(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True)
    res = result.stdout
    return '/ros_robot_controller/button' in res

while not check_node_status():
    time.sleep(0.01)

# === Start Button Listener Thread ===
button_thread = threading.Thread(target=listen_to_button_events, daemon=True)
button_thread.start()

#set both red
board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
print(" Robot ready. Waiting for Button 1...")

# === Wait for Button Press ===
while not button_pressed:
    time.sleep(0.01)
board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# Constants
pw_BLDC = 1615
pw_BLDC_stop = 1500
center_servo = 1370

min_servo = 1240
max_servo = 1500

min_bldc = 1300
max_bldc = 1700

# Proportional gain constant (tune this)
Kp = 0.075

# Color thresholds
lower_black = np.array([0, 0, 0])
upper_black = np.array([90, 60, 60])

lower_orange = np.array([0, 40, 130])
upper_orange = np.array([60, 160, 255])

lower_blue = np.array([100, 0, 0])
upper_blue = np.array([255, 80, 80])
# Regions of interest
roiLeft = (20, 260, 200, 50)
roiRight = (440, 260, 200, 50)
roiOrange = (220, 240, 240, 50)
roiBlue = (220, 240, 240, 50)
# Start motors
board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
board.pwm_servo_set_position(0.1, [[4, center_servo]])

orange = 0
line_detected = False
last_orange_time = time.time()

blue = 0
line_detected = False
last_blue_time = time.time()

cooldown_seconds = 1.5

while True:
    frame = picam2.capture_array()

    left_area = 0
    right_area = 0

    # Draw ROI rectangles
    cv2.rectangle(frame, (roiLeft[0], roiLeft[1]), (roiLeft[0]+roiLeft[2], roiLeft[1]+roiLeft[3]), (0, 255, 255), 2)
    cv2.rectangle(frame, (roiRight[0], roiRight[1]), (roiRight[0]+roiRight[2], roiRight[1]+roiRight[3]), (0, 255, 255), 2)

    # Process Left and Right ROIs
    for idx, (x, y, w, h) in zip(["Left", "Right"], [roiLeft, roiRight]):
        roi = frame[y:y+h, x:x+w]
        mask = cv2.inRange(roi, lower_black, upper_black)
        full_mask = cv2.inRange(frame, lower_black, upper_black)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        total_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                offset_cnt = cnt + [x, y]
                cv2.drawContours(frame, [offset_cnt], -1, (0, 255, 0), 2)
                total_area += area

        if idx == "Left":
            left_area = total_area
        else:
            right_area = total_area

    # === Proportional Control ===
    error = left_area - right_area
    correction = int(Kp * error)
    new_servo_pw = center_servo + correction
    new_servo_pw = max(min_servo, min(max_servo, new_servo_pw))  # Clamp

    if left_area + right_area > 0:
        #print(f"P-Control: error={error}, correction={correction}, servo={new_servo_pw}")
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
        board.pwm_servo_set_position(0.1, [[4,new_servo_pw]])
    else:
        #print("stop")
        board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])

    # === Orange Line Detection ===
    cv2.rectangle(frame, (roiOrange[0], roiOrange[1]), (roiOrange[0]+roiOrange[2], roiOrange[1]+roiOrange[3]), (0, 255, 255), 2)
    roi2 = frame[roiOrange[1]:roiOrange[1]+roiOrange[3], roiOrange[0]:roiOrange[0]+roiOrange[2]]
    orange_mask = cv2.inRange(roi2, lower_orange, upper_orange)
    contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    orange_pixels = cv2.countNonZero(orange_mask)

    if orange_pixels > 100:
        if not line_detected and (time.time() - last_orange_time) > cooldown_seconds:
            orange += 1
            last_orange_time = time.time()
            print(f'{orange} orange lines detected.')
            line_detected = True
        else:
            line_detected = False
        if orange >= 13:
            board.pwm_servo_set_position(0.1, [[2, pw_BLDC_stop]])
            break
        
    cv2.rectangle(frame, (roiBlue[0], roiBlue[1]), (roiBlue[0]+roiBlue[2], roiBlue[1]+roiBlue[3]), (0, 255, 255), 2)
    roi2 = frame[roiBlue[1]:roiBlue[1]+roiBlue[3], roiBlue[0]:roiBlue[0]+roiBlue[2]]
    blue_mask = cv2.inRange(roi2, lower_blue, upper_blue)
    contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_pixels = cv2.countNonZero(blue_mask)

    if blue_pixels > 80:
        if not line_detected and (time.time() - last_blue_time) > cooldown_seconds:
            blue += 1
            last_blue_time = time.time()
            print(f'{blue} blue lines detected.')
            line_detected = True
        else:
            line_detected = False
        if blue >= 13:
            board.pwm_servo_set_position(0.1, [[2, pw_BLDC_stop]])
            break


    # === Debug Output ===
    if debugMode == 1:
        cv2.imshow("Contours", frame)
        cv2.imshow("Full Mask", full_mask)
        cv2.imshow("Orange Mask", orange_mask)
        cv2.imshow("Blue Mask", blue_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

# Cleanup
board.pwm_servo_set_position(0.1, [[2, pw_BLDC_stop]])
picam2.stop()

