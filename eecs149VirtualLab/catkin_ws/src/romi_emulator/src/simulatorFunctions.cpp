#include "simulatorFunctions.h"

#include <geometry_msgs/Quaternion.h>

#include <cmath>
#define wheel_distance 0.15

#include "globals.h"
#include "ros/ros.h"

// Offsets to help force syncronization
double timer_offsets[] = {143223.0, 3000.0, -500.0, 0.0};
double sync_offsets[] = {123.0, 55.0, 3000.0, 1249.0};

int robot_num() { return (ros::this_node::getName().at(5) - '0') - 1; }

// Waits for the server to send its time
double waitForServerTime() {
  while (ros::Time::now().toSec() == 0) continue;
  double return_time =
      ros::Time::now().toSec() + 10000 - sync_offsets[robot_num()];

  printf("rob_num: %d, actual: %f, offset: %f, ret: %f\n", robot_num(),
         ros::Time::now().toSec(), sync_offsets[robot_num()], return_time);
  return return_time;
}

// Retrieves the current time from the server once connection has been made,
// setting the send time as well.
double waitForServerResponse(double* t_3e) {
  *t_3e = currentTime();
  double return_time =
      ros::Time::now().toSec() + 10000 + sync_offsets[robot_num()];
}

// Returns the current time in seconds, adding in an offset to require
// syncronization
double currentTime() {
  return ros::Time::now().toSec() + 10000 + timer_offsets[robot_num()];
}

double illegal_actual_time() { return ros::Time::now().toSec() + 10000; }

void globalPositionPoll(robot_position_t* positions) {
  for (int i = 0; i < NUM_ROBOTS; i++) {
    (positions + i)->x_pos = pose_data[i].position.x;
    (positions + i)->y_pos = pose_data[i].position.y;
    (positions + i)->z_pos = pose_data[i].position.z;
  }
}

void globalAnglesPoll(double* angles) {
  for (int i = 0; i < NUM_ROBOTS; i++) {
    // Compute yaw from quarternion

    geometry_msgs::Quaternion orientation = pose_data[i].orientation;

    double siny_cosp =
        2 * (orientation.w * orientation.z + orientation.x * orientation.y);

    double cosy_cosp =
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);

    angles[i] = std::atan2(siny_cosp, cosy_cosp);
  }
}

void display_write(const char* format, display_line line) {
  printf("%s", format);
}

lsm9ds1_measurement_t lsm9ds1_read_accelerometer() { return globalAccel; }

void kobukiSensorPoll(KobukiSensors_t* sensors) {
  sensors->cliffLeft = newSensors.cliffLeft;
  sensors->cliffCenter = newSensors.cliffCenter;
  sensors->cliffRight = newSensors.cliffRight;
  sensors->bumps_wheelDrops.bumpLeft = newSensors.bumps_wheelDrops.bumpLeft;
  sensors->bumps_wheelDrops.bumpCenter = newSensors.bumps_wheelDrops.bumpCenter;
  sensors->bumps_wheelDrops.bumpRight = newSensors.bumps_wheelDrops.bumpRight;
  sensors->button_pressed = newSensors.button_pressed;
  sensors->leftWheelEncoder = newSensors.leftWheelEncoder;
  sensors->rightWheelEncoder = newSensors.rightWheelEncoder;
}

void nrf_delay_ms(uint32_t delay) { ros::Duration(delay / 1000.0).sleep(); }

uint32_t lsm9ds1_start_gyro_integration() {
  if (integrateGyro) {
    // Return ret_code_t (uint32_t) = NRF_ERROR_INVALID_STATE (0x8)
    return 0x8;
  }

  integrateGyro = true;
  globalAng.x_axis = 0;
  globalAng.y_axis = 0;
  globalAng.z_axis = 0;

  // Return ret_code_t (uint32_t) = NRF_SUCCESS (0x0)
  return 0x0;
}

bool is_button_pressed(KobukiSensors_t* sensors) {
  if (sensors->button_pressed) {
    sensors->button_pressed = false;
    newSensors.button_pressed = false;
    return true;
  } else {
    return false;
  }
}

lsm9ds1_measurement_t lsm9ds1_read_gyro_integration() { return globalAng; }

// Currently does not support driving with wheel speeds set to different values
// If you implement driving with different wheel speeds, you will also need to
// account for different left/right encoder values
int32_t kobukiDriveDirect(float line_speed, float angular_speed) {
  float CmdSpeed;
  float CmdAngular;
  CmdSpeed = line_speed / 1000.0;
  CmdAngular = angular_speed;

  geometry_msgs::Twist twist;
  twist.linear.x = CmdSpeed;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = CmdAngular;
  pub.publish(twist);

  return 1;
}

void lsm9ds1_stop_gyro_integration() {
  integrateGyro = false;
  return;
}
