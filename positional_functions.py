from motion_helper_functions import *


def move_tray_to_block_number(num):
    if num == 1:
        move_to = 2
    elif 10 >= num > 1:
        move_to = (45 * (num-1)) + 2

    move_block_tray_to(move_to)


def move_pickup_gantry_to_block_pickup_pos():
    move_block_pickup_gantry_to(312)


def move_pickup_gantry_to_block_rotate_pos():
    move_block_pickup_gantry_to(167)


def move_pickup_gantry_to_block_holder_pos():
    move_block_pickup_gantry_to(7)




