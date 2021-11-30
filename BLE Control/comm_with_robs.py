# winpty /c/Users/jorda/AppData/Local/Programs/Python/Python37-32/python.exe comm_with_robs.py

import asyncio
import struct
import time
from typing import List

from bleak import BleakClient, BleakError

from ble_utils import parse_ble_args, handle_sigint
# args = parse_ble_args(
#     'Communicates with buckler display writer characteristic')
addr = "c0:98:e5:49:98:7"
robot_nums = ['1', '2', '3', '4']
addrs = [addr + robot_num for robot_num in robot_nums]
timeout = 10
handle_sigint()

DISPLAY_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DISPLAY_CHAR_UUID = "32e6108a-2b22-4db5-a914-43ce41986c70"

LAB11 = 0x02e0

class robot_data:
    x_pos:float = 0.0
    y_pos:float = 0.0
    angle:float = 0.0

class shared_data_t:
    start = time.time()
    timestamp:float = 0.0
    rob_data: List[robot_data] = []
    num_robots = 0
    packed_bytes:bytearray
    def __init__(self, num_robots):
        self.num_robots = num_robots
        for _ in range(num_robots):
            self.rob_data.append(robot_data())
    
    def update_robot(self, robot_num, x_pos, y_pos, angle):
        self.rob_data[robot_num].x_pos = x_pos
        self.rob_data[robot_num].y_pos = y_pos
        self.rob_data[robot_num].angle = angle
    
    def push_update(self):
        self.timestamp = time.time()
        data_arr = [self.timestamp]
        for data in self.rob_data:
            data_arr.extend([data.x_pos, data.y_pos, data.angle])
        self.packed_bytes =  bytearray(struct.pack("d"*(self.num_robots*3 + 1), *data_arr))


async def connect_to_device(address: str, shared_data: shared_data_t):
    while True:
        print(f"searching for device {address} ({timeout}s timeout)")
        try:
            async with BleakClient(address) as client:
                print(
                    f"Connected to device {client.address}: {client.is_connected}")
                try:
                    print("Type message and send with Enter key")
                    last = 0
                    shared_data = shared_data_t(4)
                    time.sleep(5)
                    shared_data.update_robot(0, 0.5, 0.5, 0.3)
                    shared_data.update_robot(1, -0.5, 5, 0.9)
                    shared_data.update_robot(2, 1.5, 50, 1.2)
                    shared_data.update_robot(3, -1.5, 500, 0)
                    array = shared_data.push_update()
                    print(array)
                    while True:
                        if shared_data.timestamp > last:
                            last = shared_data.timestamp
                            print("Sending packed bytes to {client.address}")
                            await client.write_gatt_char(DISPLAY_CHAR_UUID, shared_data.packed_bytes)
                except Exception as e:
                    print(f"\t{e}")
        except BleakError as e:
            print("not found")

async def begin_communication(addresses):
    shared = shared_data_t(4)
    pos_routines = [connect_to_device(address, shared) for address in addresses]
    await asyncio.gather(*pos_routines)

if __name__ == "__main__":
    # asyncio.run(begin_communication(addrs))
    
