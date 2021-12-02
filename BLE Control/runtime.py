import asyncio
import time
from comm_with_robs import begin_communication

num_robots = 4


async def main():
    shared_data, comm_tasks = await begin_communication(num_robots)
    while True:
        for i in range(4):
            shared_data.update_robot(
                i, shared_data.rob_data[i].x_pos + 0.1, shared_data.rob_data[i].y_pos + 0.1, shared_data.rob_data[i].angle + 0.1)
            shared_data.push_update()
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
