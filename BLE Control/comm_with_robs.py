# winpty /c/Users/jorda/AppData/Local/Programs/Python/Python37-32/python.exe comm_with_robs.py

import asyncio

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

class shared_data:
    timestamp = 0
    x_pos = []
    y_pos = []
    angles = []
    def __init__(self, num_robots):
        self.x_pos = [0.0]*num_robots
        self.y_pos = [0.0]*num_robots
        self.angles = [0.0]*num_robots


async def connect_to_device(address: str, shared: shared_data):
    while True:
        print(f"searching for device {address} ({timeout}s timeout)")
        try:
            async with BleakClient(address) as client:
                print(
                    f"Connected to device {client.address}: {client.is_connected}")
                try:
                    print("Type message and send with Enter key")
                    last = 0
                    while True:
                        if shared_data.timestamp != last:
                            data_arr = shared_data.x_pos + shared_data.y_pos + shared_data.angles
                            array = bytearray(data_arr)
                            display = input("")
                            await client.write_gatt_char(DISPLAY_CHAR_UUID, bytes(display, 'utf-8'))
                except Exception as e:
                    print(f"\t{e}")
        except BleakError as e:
            print("not found")

async def begin_communication(addresses):
    shared = shared_data(4)
    pos_routines = [connect_to_device(address, shared) for address in addresses]
    await asyncio.gather(*pos_routines)

if __name__ == "__main__":
    asyncio.run(begin_communication(addrs))
