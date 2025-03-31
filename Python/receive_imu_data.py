import serial
from serial.tools import list_ports
import struct
import numpy as np
from anoptv8 import *

client = ANO_CLIENT()

def find_serial_port():
    available_ports = list_ports.comports()
    if not available_ports:
        print("No available serial ports found")
        return None
    first_port = available_ports[0]
    print(f"Using Serial Port: {first_port.device}")
    try:
        ser = serial.Serial(first_port.device, baudrate=115200, timeout=1)
        print("Serial Port Opened")
        return ser
    except serial.SerialException as e:
        print(f"Error Opening Serial Port: {e}")
        return None

data_list = []
last_index = 0
def parse_imu_data(data):
    global time_stamp_list, gyro_data_list, acc_data_list, last_index
    if data[0] != 0xD1 or data[31] != 0x51:
        return False
    wkfly_timestamp = struct.unpack('<L', data[1:5])[0]
    wkfly_index = struct.unpack('<L', data[5:9])[0]
    d_index = wkfly_index - last_index
    if d_index > 1:
        # print(f"Lost {d_index-1} Data Packets")
        pass
    last_index = wkfly_index

    acc_raw = struct.unpack('<hhh', data[9:15])
    gyro_raw = struct.unpack('<hhh', data[15:21])
    
    client.SendInertialSensorData(1, acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2], 0)
    
    # print(acc_raw)
    # print(gyro_raw)
    # print(f"Timestamp: {wkfly_timestamp}, Index: {wkfly_index}")

    return True

serial_port = find_serial_port()
if serial_port:
    data_packet_size = 32
    while True:
        data = serial_port.read(data_packet_size)
        if len(data) == data_packet_size:
            if parse_imu_data(data):
                pass
            else:
                print("Invalid Data Received")
                serial_port.reset_input_buffer()
        else:
            print("Invalid Data Received")
            serial_port.reset_input_buffer()
        
        # Check Data
        # data_len = len(data_list)
        # if data_len > 2000:
        #     print("Data List Length:", data_len)
        #     dara_list_np = np.array(data_list)
        #     np.save("imu_data.npy", dara_list_np)
        #     break 
    # print("Data Saved")
    serial_port.close()
