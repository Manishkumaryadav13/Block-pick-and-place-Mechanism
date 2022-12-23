import serial
from serial.tools import list_ports
import time
import os
import platform

port_list = []
baudrate = 115200

dynamixelController_flag = None
motorController_flag = None
dynamixelController = None
motorController = None
PORT_NAME = None

OP_SYSTEM = platform.system()
if OP_SYSTEM != 'Windows':
    PORT_NAME = "ACM"
else:
    PORT_NAME = "COM"

print(PORT_NAME)
com_ports = serial.tools.list_ports.comports(include_links=True)
for i in com_ports:
    port_list.append(i.device)
print(port_list)
for i in range(len(port_list)):
    if PORT_NAME in port_list[i]:
        conn = serial.Serial(port_list[i], baudrate, timeout=1)
        time.sleep(2)
        # Get Board Name i.e Loader/Scanner/Pico
        conn.flush()
        conn.flushInput()
        conn.write("u2\n".encode())
        name = conn.readline().decode()
        conn.flush()
        conn.flushInput()
        conn.close()
    if name == "DynamixelController\r\n":
        dynamixelController = serial.Serial(port_list[i], baudrate, timeout=1)
        print("Dynamixel Control Board connected to " + str(port_list[i]))
        dynamixelController_flag = True
    elif name == "MicroTome\r\n":
        motorController = serial.Serial(port_list[i], baudrate, timeout=1)
        print("Block_changer controller board connected to " + str(port_list[i]))
        motorController_flag = True
    else:
        continue


if dynamixelController_flag and motorController_flag:
    print("Connection established successfully!")
else:
    error_msg = ""
    if dynamixelController_flag is not True:
        error_msg = error_msg + " Dynamixel controller Board Not connected "
    if motorController_flag is not True:
        error_msg = error_msg + "Blade Changer controller board not connected "
    print("Connection could not be established, " + error_msg)

