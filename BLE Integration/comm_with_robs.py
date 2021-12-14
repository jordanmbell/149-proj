# winpty /c/Users/jorda/AppData/Local/Programs/Python/Python37-32/python.exe comm_with_robs.py

import asyncio
import math
import signal
import struct
import time
from typing import List

from bleak import BleakClient, BleakError
from numpy import MAXDIMS, number

from trace_data import trace_data_t

timeout = 10
addr = "c0:98:e5:49:98:7"
ROBOT_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
POS_CHAR_UUID = "32e6108a-2b22-4db5-a914-43ce41986c70"

MESSAGES_PER_SECOND = 0.01
MAX_COMMANDS = 8


class shared_data_t:
    class robot_data:
        x_pos: float = 0.0
        y_pos: float = 0.0
        angle: float = 0.0
    disconnect = False
    connected = 0
    start = time.time()
    timestamp: float = 0.0
    rob_data: List[robot_data] = []
    num_robots = 0
    trace_data: trace_data_t = None
    start_moving_time = None
    packed_bytes: bytearray

    def __init__(self, num_robots):
        self.num_robots = num_robots
        for _ in range(num_robots):
            self.rob_data.append(self.robot_data())

    def update_robot(self, robot_num, x_pos, y_pos, angle):
        print(f'before: {robot_num}, {x_pos}, {y_pos}')
        if robot_num == 0  or robot_num == 2:
            x_pos = x_pos + 0.032 * math.sin(angle) - 0.05 * math.cos(angle)
            y_pos = y_pos - 0.032 * math.cos(angle) - 0.05 * math.sin(angle)
        else:
            x_pos = x_pos + 0.032 * math.sin(angle) + 0.05 * math.cos(angle)
            y_pos = y_pos - 0.032 * math.cos(angle) + 0.05 * math.sin(angle)
        print(f'after: {robot_num}, {x_pos}, {y_pos}')
        self.rob_data[robot_num].x_pos = x_pos
        self.rob_data[robot_num].y_pos = y_pos
        self.rob_data[robot_num].angle = angle

    def update_trace_data(self, trace_data, start_moving_time):
        self.trace_data = trace_data
        self.start_moving_time = start_moving_time

    def push_update(self):
        self.timestamp = time.time() - self.start
        data_arr = [self.timestamp]
        # Push Robot data
        for data in self.rob_data:
            data_arr.extend([data.x_pos, data.y_pos, data.angle])

        # Push Trace Data
        num_commands = len(self.trace_data.cmds)
        data_arr.append(num_commands)
        for data in self.trace_data.cmds:
            data_arr.append(data)
        # Fill in blank spots
        for _ in range(num_commands, MAX_COMMANDS):
            data_arr.append(0)
        for i in range(num_commands):
            cmd = self.trace_data.cmds[i]
            if cmd == 0: # Straight command
                data_arr.append(self.trace_data.cmd_param_dist[i])
            else:
                data_arr.append(self.trace_data.cmd_param_angle[i])
        # Fill in blank spots
        for _ in range(num_commands, MAX_COMMANDS):
            data_arr.append(0)
        for data in self.trace_data.cmd_times:
            data_arr.append(data)
        # Fill in blank spots
        for _ in range(num_commands, MAX_COMMANDS):
            data_arr.append(0)
        data_arr.append(self.start_moving_time)
        formt_string = "f" * (1 + len(self.rob_data)*3) + "i" * (MAX_COMMANDS + 1) + "f" * (MAX_COMMANDS * 2 + 1)

        self.packed_bytes = bytearray(struct.pack(formt_string, *data_arr))
        # print(self.packed_bytes)


async def _connect_to_device(address: str, shared_data: shared_data_t, rob_number: number):
    while not shared_data.disconnect:
        print(f"searching for device {address} ({timeout}s timeout)")
        try:
            async with BleakClient(address) as client:
                print(
                    f"Connected to device {client.address}: {client.is_connected}")
                shared_data.connected += 1
                try:
                    last = 0
                    last_x = 0
                    last_y = 0
                    while not shared_data.disconnect:
                        if shared_data.timestamp > last and last_x != shared_data.rob_data[rob_number].x_pos and last_y != shared_data.rob_data[rob_number].y_pos:
                            last_x = shared_data.rob_data[rob_number].x_pos
                            last_y = shared_data.rob_data[rob_number].y_pos
                            last = shared_data.timestamp
                            await client.write_gatt_char(POS_CHAR_UUID, shared_data.packed_bytes)
                            print(last)
                            # Rate limit
                            await asyncio.sleep(1 / MESSAGES_PER_SECOND)
                        else:
                            await asyncio.sleep(0)  # Allow other events to run
                    await client.disconnect()
                except Exception as e:
                    print(f"\t{e}")
        except Exception as e:
            print(f"Did not find device {address} : {e}")
        await asyncio.sleep(0)  # Allow other events to run


def handle_sigint(comm_tasks, shared: shared_data_t):
    def signal_handler(sig, fram):
        shared.disconnect = True
        print("Disconnecting bluetooth")
    signal.signal(signal.SIGINT, signal_handler)



async def begin_communication(num_robots):
    addresses = [addr + str(num) for num in range(num_robots)]
    shared = shared_data_t(4)
    pos_routines = [_connect_to_device(address, shared, i)
                    for i, address in enumerate(addresses)]
    return shared, asyncio.gather(*pos_routines)
