import asyncio
import time
from comm_with_robs import begin_communication

num_robots = 4

async def main():
    addr = "c0:98:e5:49:98:7"
    robot_nums = ['1', '2', '3', '4']
    addrs = [addr + robot_num for robot_num in robot_nums]
    shared_data, comm_tasks = await begin_communication(num_robots)
    while True:
        await asyncio.sleep(0)
        
if __name__ == "__main__":
    asyncio.run(main())
