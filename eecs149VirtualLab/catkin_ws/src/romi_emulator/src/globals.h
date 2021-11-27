#include "kobukiSensorTypes.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
extern "C" {
  #include "controller.h"
}
#ifndef MY_GLOBALS_H
#define MY_GLOBALS_H

// This is a declaration of your variable, which tells the linker this value
// is found elsewhere.  Anyone who wishes to use it must include global.h,
// either directly or indirectly.
extern ros::Publisher pub;
extern lsm9ds1_measurement_t globalAccel;
extern bool bp;
extern bool cf;
extern int which_bp;
extern int which_cf;
extern int cf_dis;
extern KobukiSensors_t newSensors;
extern geometry_msgs::Point prev_pos;
extern robot_state_t state;
extern lsm9ds1_measurement_t globalAng;
extern lsm9ds1_measurement_t runningGlobalAng;
extern float prev_time;
extern bool integrateGyro;
extern bool monitors;

#define NUM_ROBOTS 4
#define POSE_UPDATE_HZ 10
extern geometry_msgs::Pose recent_poses[NUM_ROBOTS];
extern geometry_msgs::Pose pose_data[NUM_ROBOTS];
extern int robot_idx;

#endif
