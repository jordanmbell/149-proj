#include "globals.h"
#include <geometry_msgs/Point.h>

// This is the definition of shared global variables.  It can only happen in one place.
// You must include global.h so that the compiler matches it to the correct
// one, and doesn't implicitly convert it to static.
ros::Publisher pub;
lsm9ds1_measurement_t globalAccel = {0, 0, 9.8};
bool bp = false;
bool cf = false;
int which_bp = -1;
int which_cf = -1;
int cf_dis = 0;
geometry_msgs::Point prev_pos;
KobukiSensors_t newSensors = {0};
robot_state_t state = OFF;
lsm9ds1_measurement_t globalAng = {0, 0, 0};
lsm9ds1_measurement_t runningGlobalAng = {0, 0, 0};
float prev_time = 0.0;
bool integrateGyro = false;

// Set to true if running with monitors to enable logging
bool monitors = false;

geometry_msgs::Pose recent_poses[NUM_ROBOTS];
geometry_msgs::Pose pose_data[NUM_ROBOTS];
int robot_idx = 0;