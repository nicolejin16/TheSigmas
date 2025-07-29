import sys
import time
import ros_robot_controller_sdk as rrc
from readchar import readkey

board = rrc.Board()

# Initial pulse width values
pw_BLDC = 1500  # Stop
angle = 90      # Degrees
pw_Servo = int(angle * 11.1 + 500)  # Convert angle to pulse width

# Initialize ESC (channel 2) and servo (channel 1)
board.pwm_servo_set_position(0.1, [[2, pw_BLDC]])
board.pwm_servo_set_position(0.1, [[1, pw_Servo]])
time.sleep(6)  # Wait for ESC to initialize

# Limits
MIN_ANGLE = 45
MAX_ANGLE = 135
ANGLE_STEP = 1

MIN_BLDC = 1300
MAX_BLDC = 1700
BLDC_STEP = 10

def update_servo(angle):
    pw = int(angle * 11.1 + 500)
    board.pwm_servo_set_position(0.1, [[1, pw]])
    print(f"Steering angle: {angle}°, PWM: {pw}")

def update_motor(pw):
    board.pwm_servo_set_position(0.1, [[2, pw]])
    print(f"Motor PWM: {pw}")

if __name__ == '__main__':
    print("Controls: [A] left | [D] right | [S] backward | [W] forward | [space] stop | [Q] quit")
    try:
        while True:
            k = readkey()
            if k in ['a', 'A']:
                if angle > MIN_ANGLE:
                    angle -= ANGLE_STEP
                    update_servo(angle)

            elif k in ['d', 'D']:
                if angle < MAX_ANGLE:
                    angle += ANGLE_STEP
                    update_servo(angle)

            elif k in ['w', 'W']:
                if pw_BLDC < MAX_BLDC:
                    pw_BLDC += BLDC_STEP
                    update_motor(pw_BLDC)

            elif k in ['s', 'S']:
                if pw_BLDC > MIN_BLDC:
                    pw_BLDC -= BLDC_STEP
                    update_motor(pw_BLDC)

            elif k == ' ':
                pw_BLDC = 1500
                angle = 90
                update_motor(pw_BLDC)
                update_servo(angle)
                print("Stopped and centered.")

            elif k in ['q', 'Q']:
                print("Exiting...")
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Interrupted. Stopping.")
        board.pwm_servo_set_position(0.1, [[2, 1500]])
        board.pwm_servo_set_position(0.1, [[1, 1500]])
