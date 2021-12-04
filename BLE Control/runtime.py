import asyncio
from comm_with_robs import begin_communication, handle_sigint

num_robots = 1


async def main():
    shared_data, comm_tasks = await begin_communication(num_robots)
    handle_sigint(comm_tasks, shared_data)
    # comm_tasks are now running asyncronously, from this thread perform 
    # the sensor and data update tasks.
    # Use shared_data.update_robot() to update data, and push with push_update
    while not shared_data.disconnect:
        if shared_data.connected:
            for i in range(4):
                shared_data.update_robot(
                    i, shared_data.rob_data[i].x_pos + 0.1, shared_data.rob_data[i].y_pos + 0.1, shared_data.rob_data[i].angle + 0.1)
            shared_data.push_update()
        await asyncio.sleep(0.1)
    print("Event loop stopped")
    await comm_tasks
    print("Finished cleanup")

if __name__ == "__main__":
    asyncio.run(main())
