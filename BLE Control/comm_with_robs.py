# winpty /c/Users/jorda/AppData/Local/Programs/Python/Python37-32/python.exe comm_with_robs.py

import asyncio
import struct
import time
from typing import List

from bleak import BleakClient, BleakError

from ble_utils import handle_sigint
timeout = 10
handle_sigint()

addr = "c0:98:e5:49:98:7"
ROBOT_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
POS_CHAR_UUID = "32e6108a-2b22-4db5-a914-43ce41986c70"


class shared_data_t:
    class robot_data:
        x_pos: float = 0.0
        y_pos: float = 0.0
        angle: float = 0.0
    connected = 0
    start = time.time()
    timestamp: float = 0.0
    rob_data: List[robot_data] = []
    num_robots = 0
    packed_bytes: bytearray

    def __init__(self, num_robots):
        self.num_robots = num_robots
        for _ in range(num_robots):
            self.rob_data.append(self.robot_data())

    def update_robot(self, robot_num, x_pos, y_pos, angle):
        self.rob_data[robot_num].x_pos = x_pos
        self.rob_data[robot_num].y_pos = y_pos
        self.rob_data[robot_num].angle = angle

    def push_update(self):
        self.timestamp = time.time() - self.start
        data_arr = [self.timestamp]
        for data in self.rob_data:
            data_arr.extend([data.x_pos, data.y_pos, data.angle])
        self.packed_bytes = bytearray(struct.pack(
            "d"*(self.num_robots*3 + 1), *data_arr))


async def _connect_to_device(address: str, shared_data: shared_data_t):
    while True:
        print(f"searching for device {address} ({timeout}s timeout)")
        try:
            async with BleakClient(address) as client:
                print(
                    f"Connected to device {client.address}: {client.is_connected}")
                shared_data.connected += 1
                try:
                    last = 0
                    while True:
                        if shared_data.timestamp > last:
                            last = shared_data.timestamp
                            await client.write_gatt_char(POS_CHAR_UUID, shared_data.packed_bytes)
                            print(last)
                        await asyncio.sleep(0)  # Allow other events to run
                except Exception as e:
                    print(f"\t{e}")
        except BleakError as e:
            print("not found")
        await asyncio.sleep(0)  # Allow other events to run


async def begin_communication(num_robots):
    addresses = [addr + str(num+1) for num in range(num_robots)]
    shared = shared_data_t(4)
    pos_routines = [_connect_to_device(address, shared)
                    for address in addresses]
    return shared, asyncio.gather(*pos_routines)
