#include "simulatorFunctions.h"
#include "globals.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#define wheel_distance 0.15


void kobukiPositionPoll(robot_position_t* positions) {
  for (int i = 0; i < NUM_ROBOTS; i ++) {
    (positions + i)->x_pos = pose_data[i].position.x;
    (positions + i)->y_pos = pose_data[i].position.y;
    (positions + i)->z_pos = pose_data[i].position.z;
  }
}

void display_write(const char *format, display_line line) {
  printf("%s", format);
}

lsm9ds1_measurement_t lsm9ds1_read_accelerometer() {
  return globalAccel;
}

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

void nrf_delay_ms(uint32_t delay) {
  ros::Duration(delay/1000.0).sleep();
}

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

lsm9ds1_measurement_t lsm9ds1_read_gyro_integration() {
  return globalAng;
}

// Currently does not support driving with wheel speeds set to different values
// If you implement driving with different wheel speeds, you will also need to
// account for different left/right encoder values
int32_t kobukiDriveDirect(float leftWheelSpeed, float rightWheelSpeed, float set_radius) {
  float CmdSpeed;
  float CmdAngular;
  CmdSpeed = ((leftWheelSpeed + rightWheelSpeed) / 2.0) / 1000.0;
  if (set_radius == 0) {
    CmdAngular = 2*rightWheelSpeed/wheel_distance;
  } else if (rightWheelSpeed > leftWheelSpeed) {
    CmdAngular = CmdSpeed/set_radius;
  } else if (rightWheelSpeed == leftWheelSpeed) {
    CmdAngular = 0;
  } else {
    CmdAngular = -CmdSpeed/set_radius;
  }

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
