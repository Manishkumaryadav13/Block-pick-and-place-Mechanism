from positional_functions import *
import time
def process():
    home_all()
    time.sleep(2)
    move_pickup_gantry_to_block_pickup_pos()
    time.sleep(2)
    open_gripper()
    time.sleep(2)
    gripper_to_loading_rack()
    time.sleep(2)
    close_gripper()
    time.sleep(2)
    gripper_to_loading_above_rack()
    time.sleep(2)
    move_pickup_gantry_to_block_rotate_pos()
    time.sleep(10)
    block_unclamping()
    time.sleep(2)
    gripper_to_loading_rotating_block()
    time.sleep(4)
    block_clamping()
    time.sleep(2)
    open_gripper()
    time.sleep(2)
    gripper_to_loading_above_rack()
    time.sleep(4)
    rotate_block_Horizontal()
    time.sleep(3)
    rotate_block_Vertical()
    time.sleep(4)
    gripper_to_loading_rotating_block()
    time.sleep(4)
    close_gripper()
    time.sleep(2)
    block_unclamping()
    time.sleep(2)
    gripper_to_loading_above_rack()
    time.sleep(4)
    move_pickup_gantry_to_block_holder_pos()
    time.sleep(10)
    open_block_holder()
    time.sleep(2)
    gripper_to_loading_block_holder()
    time.sleep(4)
    close_block_holder()
    time.sleep(4)
    open_gripper()
    time.sleep(2)
    gripper_to_loading_above_rack()
    time.sleep(4)
    gripper_to_loading_block_holder()
    time.sleep(4)
    close_gripper()
    time.sleep(4)
    open_block_holder()
    time.sleep(4)
    gripper_to_loading_above_rack()
    time.sleep(4)
    move_pickup_gantry_to_block_pickup_pos()
    time.sleep(14)
    gripper_to_loading_rack()
    time.sleep(2)
    open_gripper()
    time.sleep(2)
    gripper_to_loading_above_rack()
    time.sleep(2)





def pick_block_from_block_tray(num):
    move_tray_to_block_number(num)
    time.sleep(3)
    move_pickup_gantry_to_block_pickup_pos()
    time.sleep(3)


def pick_block_from_block_holder():
    move_pickup_gantry_to_block_holder_pos()
    time.sleep(3)
    open_block_holder()
    time.sleep(3)
    close_block_holder()


def load_block(num):
    open_block_holder()
    pick_block_from_block_tray(num)
    move_pickup_gantry_to_block_holder_pos()
    time.sleep(3)
    close_block_holder()
    time.sleep(3)
    move_pickup_gantry_to_block_pickup_pos()


def remove_block(num):
    pick_block_from_block_holder(num)
    move_pickup_gantry_to_block_pickup_pos()
    time.sleep(3)
    move_tray_to_block_number(num)


def rotate_block():
    move_pickup_gantry_to_block_rotate_pos()
    time.sleep(3)
    move_pickup_gantry_to_block_holder_pos()
    time.sleep(3)
    open_block_holder()
    time.sleep(3)
    close_block_holder()


def home_all():
    home_block_tray_gantry()
    home_block_pickup_gantry()
    home_block_holder()
