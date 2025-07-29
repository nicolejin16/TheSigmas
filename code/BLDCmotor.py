import time
import ros_robot_controller_sdk as rrc

board = rrc.Board()

# arm the ESC
board.pwm_servo_set_position(0.1, [[2, 1500]])
time.sleep(6)

board.pwm_servo_set_position(0.1, [[2, 1350]])
time.sleep(3)
board.pwm_servo_set_position(0.1, [[2, 1500]])
time.sleep(3)
board.pwm_servo_set_position(0.1, [[2, 1750]])
time.sleep(3)
board.pwm_servo_set_position(0.1, [[2, 1350]])