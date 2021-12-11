# Concurrent programming utilities
import asyncio
from multiprocessing import Process, Manager, Value

# Robot communication
from comm_with_robs import *

# Position Animation
from gui_anim import *
#from gui_anim_3d import *

# Camera Tracking
from camera import *

# Trace construction
from trace import *

# Variables to configure -------------
num_markers = 9
num_robots = 4
marker_to_robot = {
    0:0,
    1:1,
    2:2,
    3:3
}

update_delay_s = 0.01


route_locations = [[1,-4], [4,3]]  # For now, trace locations set by user
turn_radius = 1.5  # meter
speed = 0.01         # meter/sec
angular_speed = 10  # deg /sec
setup_time = 4
starting_orientation = np.array([0,1])

open_ble = True

delta_moving_time = 10    # Set when the robots should start moving after connection
# End of variables to configure -------------

# Main server loop
async def main():
    # BLE Connection setup
    if open_ble:
        shared_data, comm_tasks = await begin_communication(num_robots)
        handle_sigint(comm_tasks, shared_data)
        shared_data.disconnect = False
    else:
        shared_data = shared_data_t(4)
        handle_sigint(None, shared_data)
        shared_data.disconnect = False
        shared_data.connected = num_robots



    # Multiprocess manager to share marker coordinates across processes
    with Manager() as manager:
        shared_marker_dict = manager.dict()
        shared_is_calibrated = Value('i', 0)
        for i in range(num_markers):
            shared_marker_dict[i] = [0,0,0]

        # ArUco Tracker Process
        p_track = Process(target=track, args=(shared_marker_dict, shared_is_calibrated,))
        p_track.start()

        # Calculate trace given positions of markers or user defined coordinates
        trace_data = trace_data_t()
        #trace_data.route_with_orientation(route_locations, turn_radius, speed, angular_speed, setup_time, starting_orientation)

        # Graph markers Process
        p_graph = Process(target=start_animation, args=(shared_marker_dict, num_markers, trace_data, route_locations, starting_orientation))
        p_graph.start()

        # Wait for BLE connections to finish before sending trace and robot data
        # while not shared_data.connected == num_robots:
        #     if shared_data.disconnect:
        #         break
        #     print("Waiting for BLE connection before pushing trace data")
        #     await asyncio.sleep(5)
        # start_moving_time = delta_moving_time + time.time() - shared_data.start
        # print("BLE connected to all robots: pushing trace data and start time: ")
        # print(trace_data)
        # print(start_moving_time)
        # shared_data.update_trace_data(trace_data, start_moving_time)
        # shared_data.push_update()

        # Wait for camera to be shared_is_calibrated
        while not shared_is_calibrated.value:
            if shared_data.disconnect:
                break
            print("Server waiting for cameras to calibrate")
            await asyncio.sleep(5)

        if shared_is_calibrated.value:
            await asyncio.sleep(1)
            print("Server now has calibrated camera values")

        # Translate robot coordinates so origin is at the package
        delta_x, delta_y, delta_rot = 0,0,0
        sines = 0
        cosines = 0
        for i in range(num_robots):
            rot = shared_marker_dict[i][2]
            # print("Rotation: ", rot)
            sines += np.sin(np.radians(rot))
            cosines += np.cos(np.radians(rot))
            delta_x += shared_marker_dict[i][0] / num_robots
            delta_y += shared_marker_dict[i][1] / num_robots
            # delta_rot += rot

        delta_rot = (delta_rot % 360) / num_robots
        delta_rot = np.degrees(np.arctan(sines/cosines))
        print("Delta rot", delta_rot)

        # Update shared date for BLE clients
        while not shared_data.disconnect:
            if shared_data.connected:
                change = False
                for i in range(num_robots):
                    if i in shared_marker_dict.keys():
                        x = shared_marker_dict[i][0] - delta_x
                        y = shared_marker_dict[i][1] - delta_y
                        rot = shared_marker_dict[i][2] - delta_rot
                        # print("updating ", i, ", x = ",x," y = ", y, " rot = ", rot)
                        shared_data.update_robot(marker_to_robot[i],
                                x, y, rot)
                        change = True
                if change:
                    shared_data.push_update()

            await asyncio.sleep(update_delay_s)

        # Main loop finished, await communication shutdown
        print("Shutdown communication")
        shared_data.disconnect = True
        if open_ble:
            await comm_tasks
        print("Comm shut down complete")

        # Clean up processes
        print("Waiting to close camera")
        p_track.join()
        print("Waiting to close matplotlib graph")
        p_graph.join()
        print("Finished multiprocess cleanup")
        print("All tasks finished")


if __name__=="__main__":
    asyncio.run(main())
