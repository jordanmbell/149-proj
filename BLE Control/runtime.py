import asyncio
from comm_with_robs import begin_communication


async def main():
    addr = "c0:98:e5:49:98:7"
    robot_nums = ['1', '2', '3', '4']
    addrs = [addr + robot_num for robot_num in robot_nums]
    asyncio.create_task(begin_communication((addrs)))

if __name__ == "__main__":
    asyncio.run(main())
