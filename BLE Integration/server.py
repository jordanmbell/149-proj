# Threading utilities
import asyncio
from multiprocessing import Process, Manager

# Robot communication
from comm_with_robs import *

# Position Animation
from gui_anim import *

# Camera Tracking
from camera import *

# Variables to configure -------------
marker_to_robot = {
    1:1,
    2:2,
    3:3
}

# End of variables to configure -------------

# Main server loop
async def main():
    shared_data, comm_tasks = await begin_communication(4)
    handle_sigint(comm_tasks, shared_data)

    # For now, assume that everything is connected
    shared_data.disconnect = False
    shared_data.connected = True

    # Multiprocess manager to share coordinates across managers
    with Manager() as manager:
        shared_dict = manager.dict()
        for i in range(4):
            shared_dict[i] = [0,0]

        # Graph markers Process
        p_graph = Process(target=start_animation, args=(shared_dict,))
        p_graph.start()

        # ArUco Tracker Process
        p_track = Process(target=track, args=(shared_dict,))
        p_track.start()

        # Update shared date for BLE clients
        while not shared_data.disconnect:
            if shared_data.connected:
                change = False
                for i in range(4):
                    if i in marker_to_robot.keys():
                        shared_data.update_robot(marker_to_robot[i],
                                shared_dict[i][0], shared_dict[i][1], shared_dict[i][1])
                        change = True
                if change:
                    shared_data.push_update()

            await asyncio.sleep(0.1)

        # Clean up
        print("Waiting to close matplotlib graph")
        p_graph.join()
        print("Waiting to close camera")
        p_track.join()
        print("Finished cleanup")
    # Main loop finished, await communication shutdown
    print("Shutdown communication")
    shared_data.disconnect = True
    await comm_tasks
    print("All tasks finished")

if __name__=="__main__":
    asyncio.run(main())
