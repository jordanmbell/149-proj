# Concurrent programming utilities
import asyncio
from multiprocessing import Process, Manager

# Robot communication
from comm_with_robs import *

# Position Animation
from gui_anim import *
#from gui_anim_3d import *

# Camera Tracking
from camera import *

# Variables to configure -------------
marker_to_robot = {
    1:1,
    2:2,
    3:3
}

update_delay_s = 0.01
num_markers = 9
# End of variables to configure -------------

# Main server loop
async def main():
    # BLE Connection setup
    shared_data = shared_data_t(4)
    handle_sigint(None, shared_data)

    # For now, assume that everything is connected
    shared_data.disconnect = False
    shared_data.connected = True

    # Multiprocess manager to share marker coordinates across processes
    with Manager() as manager:
        shared_marker_dict = manager.dict()
        for i in range(num_markers):
            shared_marker_dict[i] = [0,0,0]

        # Graph markers Process
        p_graph = Process(target=start_animation, args=(shared_marker_dict, num_markers))
        p_graph.start()

        # ArUco Tracker Process
        p_track = Process(target=track, args=(shared_marker_dict, ))
        p_track.start()

        # Update shared data for BLE clients
        while not shared_data.disconnect:
            if shared_data.connected:
                for i in range(4):
                    if i in marker_to_robot.keys():
                        shared_data.update_robot(marker_to_robot[i],         # Robot ID
                         shared_marker_dict[i][0], shared_marker_dict[i][1], # Robot X,Y
                         shared_marker_dict[i][2])                           # Robot Rot (not implemented rn)
            await asyncio.sleep(update_delay_s)

        # Clean up
        print("Waiting to close camera")
        p_track.join()
        print("Waiting to close matplotlib graph")
        p_graph.join()
        print("Finished cleanup")

if __name__=="__main__":
    asyncio.run(main())