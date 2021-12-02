import asyncio
import time
from comm_with_robs import begin_communication

num_robots = 1


async def main():
    shared_data, comm_tasks = await begin_communication(num_robots)
    # comm_tasks are now running asyncronously, from this thread perform 
    # the sensor and data update tasks.
    # Use shared_data.update_robot() to update data, and push with push_update
    while True:
        if shared_data.connected:
            for i in range(4):
                shared_data.update_robot(
                    i, shared_data.rob_data[i].x_pos + 0.1, shared_data.rob_data[i].y_pos + 0.1, shared_data.rob_data[i].angle + 0.1)
            shared_data.push_update()
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())
