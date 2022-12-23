from serial_comm import *
import time

# Axis used for Gantry control
GANTRY_X_AXIS = "X"
GANTRY_Y_AXIS = "Y"
BLOCK_HOLDER = "Z"
DEFAULT_GANTRY_SPEED = 200
DEFAULT_GANTRY_ACCELERATION = 300
DEFAULT_BLADE_SPEED = 12
DEFAULT_BLADE_ACCELERATION = 30
HOME = "G28"
MOVE = "G0"
SPEED = "M203"
ACCELERATION = "M201"
RELATIVE_MOTION = "G91"
ABSOLUTE_MOTION = "G90"
VACUUM_ON = 255
VACUUM_OFF = 0
PUMP = "M106 P0 S"

ID ="I"
UART_1='1'
UART_0='0'
ID_1='1'
ID_2='2'
POSITION='P'
VELOCITY='V=200'
abs_pos="1000"
abs_pos1="0"




def home_block_tray_gantry():
    motorController.write((HOME + GANTRY_Y_AXIS + "\n").encode())
    time.sleep(0.1)
    print("Block Tray Homing")


def home_block_pickup_gantry():
    motorController.write((HOME + GANTRY_X_AXIS + "\n").encode())
    time.sleep(0.1)
    print("Block pickup gantry homing")


def home_block_holder():
    motorController.write((HOME + BLOCK_HOLDER + "\n").encode())
    time.sleep(0.1)
    motorController.write((HOME + BLOCK_HOLDER + "\n").encode())
    print("Block holder homing")


"""
    Relative motion functions for motion in individual axis direction
"""


# Gantry axis direction

def move_block_tray_forward_by(delta):
    _move_block_tray_by(delta)


def move_block_tray_backward_by(delta):
    _move_block_tray_by(-1 * delta)


def move_block_pickup_gantry_left_by(delta):
    _move_block_pickup_gantry_by(delta)


def move_block_pickup_gantry_right_by(delta):
    _move_block_pickup_gantry_by(-1 * delta)


def _move_block_pickup_gantry_by(delta):
    motorController.write((RELATIVE_MOTION + "\n").encode())
    motorController.write((MOVE + GANTRY_X_AXIS + str(delta) + "\n").encode())
    motorController.write((ABSOLUTE_MOTION + "\n").encode())


def _move_block_tray_by(delta):
    motorController.write((RELATIVE_MOTION + "\n").encode())
    motorController.write((MOVE + GANTRY_Y_AXIS + str(delta) + "\n").encode())
    motorController.write((ABSOLUTE_MOTION + "\n").encode())


def move_block_pickup_gantry_to(abs_pos):
    motorController.write((MOVE + GANTRY_X_AXIS + str(abs_pos) + "\n").encode())


def move_block_tray_to(abs_pos):
    motorController.write((MOVE + GANTRY_Y_AXIS + str(abs_pos) + "\n").encode())


def open_block_holder():
    motorController.write((ABSOLUTE_MOTION + "\n").encode())
    motorController.write((MOVE + BLOCK_HOLDER + "5" + "\n").encode())


def close_block_holder():
    motorController.write((ABSOLUTE_MOTION + "\n").encode())
    motorController.write((MOVE + BLOCK_HOLDER + "-5" + "\n").encode())


# speed & acc for all axes
def set_gantry_max_speed(mm_per_sec):
    motorController.write((SPEED + GANTRY_X_AXIS + str(mm_per_sec) + "\n").encode())
    motorController.write((SPEED + GANTRY_Y_AXIS + str(mm_per_sec) + "\n").encode())


def set_gantry_acc(acc):
    motorController.write((ACCELERATION + GANTRY_X_AXIS + str(acc) + "\n").encode())
    motorController.write((ACCELERATION + GANTRY_Y_AXIS + str(acc) + "\n").encode())


def set_block_holder_max_speed(mm_per_sec):
    motorController.write((SPEED + BLOCK_HOLDER + str(mm_per_sec) + "\n").encode())


def set_blade_acc(acc):
    motorController.write((ACCELERATION + BLOCK_HOLDER + str(acc) + "\n").encode())


def set_default_speed_and_acceleration():
    set_gantry_max_speed(DEFAULT_GANTRY_SPEED)
    set_gantry_acc(DEFAULT_GANTRY_ACCELERATION)
    set_block_holder_max_speed(DEFAULT_BLADE_SPEED)
    set_blade_acc(DEFAULT_BLADE_ACCELERATION)


def current_gantry_pos():
    motorController.flush()
    motorController.flushInput()
    motorController.write("M114\n".encode())
    resp = motorController.readline().decode("UTF-8")
    print(resp)

# Dynamixel


def block_clamping(clamping_pos=0):
    dynamixelController.write((ID+UART_0+ID_1+','+VELOCITY+"\n").encode())
    print((ID+UART_0+ID_1+','+VELOCITY+"\n"))
    dynamixelController.write((ID+UART_0+ID_1+','+POSITION+"="+str(clamping_pos)+"\n").encode())
    print((ID+UART_0+ID_1+','+POSITION+"="+str(clamping_pos)+"\n"))


def block_unclamping(unclamping_pos = 1000):
    dynamixelController.write((ID + UART_0 + ID_1 + ',' + VELOCITY + "\n").encode())
    print((ID + UART_0 + ID_1 + ',' + VELOCITY + "\n"))
    dynamixelController.write((ID + UART_0 + ID_1 + ',' + POSITION + "=" + str(unclamping_pos) + "\n").encode())
    print((ID + UART_0 + ID_1 + ',' + POSITION + "="+str(unclamping_pos) + "\n"))

def rotate_block_Horizontal(angle=1140):
    dynamixelController.write((ID + UART_0 + ID_2 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_0+ID_2+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) ))

def rotate_block_Vertical(angle=0):
    dynamixelController.write((ID + UART_0 + ID_2 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_0+ID_2+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) ))

def open_gripper(angle=0):
    dynamixelController.write((ID + UART_1 + ID_1 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_1+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_1+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_1+','+ POSITION+"="+str(angle) ))

def close_gripper(angle=2048):
    dynamixelController.write((ID + UART_1 + ID_1 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_1+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_1+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_1+','+ POSITION+"="+str(angle) ))

def gripper_to_loading_block(angle=1065):
    dynamixelController.write((ID + UART_1 + ID_2 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) ))

def gripper_to_loading_rack(angle=1550):
    dynamixelController.write((ID + UART_1 + ID_2 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) ))

def gripper_to_loading_above_rack(angle=2000):
    dynamixelController.write((ID + UART_1 + ID_2 + ',' + VELOCITY + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+str(angle) + "\n").encode())
    dynamixelController.write((ID+UART_1+ID_2+','+ POSITION+"="+str(angle) + "\n").encode())
    print((ID+UART_0+ID_2+','+ POSITION+"="+str(angle) ))

def current_gripper_pos():
    dynamixelController.flush()
    dynamixelController.flushInput()
    dynamixelController.write((CURRENT_GRIPPER_POS+"\n").encode())
    resp = dynamixelController.readline().decode("UTF-8")
    resp2 = dynamixelController.readline().decode("UTF-8")
    print(resp2)


def reset_dynamixel():
    dynamixelController.write((RESET_DYNAMIXEL+"\n").encode())
    resp = dynamixelController.readline().decode("UTF-8")
    print(resp)


def max_speed():
    motorController.write((MOVE + "F12000" + "\n"))



