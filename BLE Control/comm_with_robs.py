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


async def connect_to_device(address):
    print(f"searching for device {address} ({timeout}s timeout)")
    try:
        async with BleakClient(address) as client:
            print(
                f"Connected to device {client.address}: {client.is_connected}")
            sv = client.services[DISPLAY_SERVICE_UUID]
            try:
                print("Type message and send with Enter key")
                while True:
                    display = input("")
                    await client.write_gatt_char(DISPLAY_CHAR_UUID, bytes(display, 'utf-8'))
            except Exception as e:
                print(f"\t{e}")
    except BleakError as e:
        print("not found")

def main(addresses):
    routines = [connect_to_device(address) for address in addresses]
    return asyncio.gather(*routines)

if __name__ == "__main__":
    while True:
        asyncio.run(main(addrs))
