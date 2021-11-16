/** Driver code for emulating the Romi Robot using a Kobuki simulation
*   Authors:  Victoria Tuck
*             Bernard Chen
*   ----------
*   Subscribes to relevant ROS topics from the Gazebo Kobuki simulation and
*   calls controller.c robot controller for the state machine implementation
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/ModelStates.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <iomanip>
#include <ros/console.h>
#include "kobukiSensorTypes.h"
#include "simulatorFunctions.h"
#include "globals.h"
#include <math.h>
#include <sstream>
#include <string>

void processOdometry(const nav_msgs::Odometry& data) {
  if (monitors) {
    ROS_INFO("Odometry: Pose Position: (x, y, z) = (%f, %f, %f), Pose Orientation: (x, y, z, w) = (%f, %f, %f, %f), Linear Twist: (x, y, z) = (%f, %f, %f), Angular Twist: (x, y, z) = (%f, %f, %f)", 
      data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
      data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,
      data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
      data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z);
  }

  // Add encoder values
  float distance = sqrt(pow(data.pose.pose.position.x - prev_pos.x, 2) +
                      pow(data.pose.pose.position.y - prev_pos.y, 2) +
                      pow(data.pose.pose.position.z - prev_pos.z, 2));

  // Update prev_pos
  prev_pos.x = data.pose.pose.position.x;
  prev_pos.y = data.pose.pose.position.y;
  prev_pos.z = data.pose.pose.position.z;

  // TODO: Decrease encoder values if driving backwards
  // Calculate angle between orientation vector (quaternion*forward axis) and position delta vector
  // If pointing same way (abs(angle) < 90), driving forward. If not, driving backward, should subtract
  // new encoder value.

  // Convert dist to encoder values
  const float CONVERSION = 0.0006108;
  newSensors.leftWheelEncoder = newSensors.leftWheelEncoder + (distance / CONVERSION);
  newSensors.rightWheelEncoder = newSensors.rightWheelEncoder + (distance / CONVERSION);
}

void processIMU(const sensor_msgs::Imu& data) {
  if (monitors) {
    ROS_INFO("IMU Data: Orientation: (x, y, z, w) = (%f, %f, %f, %f), Angular Velocity: (x, y, z) = (%f, %f, %f), Linear Acceleration: (x, y, z) = (%f, %f, %f)",
      data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
      data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
  }

  // Kobuki
  //globalAccel.x_axis = data.linear_acceleration.x;
  //globalAccel.y_axis = data.linear_acceleration.y;
  globalAccel.z_axis = data.linear_acceleration.z;
  // Romi
  globalAccel.x_axis = data.linear_acceleration.y;
  globalAccel.y_axis = -data.linear_acceleration.x;

  float time_diff = data.header.stamp.sec + (data.header.stamp.nsec / 1000000000.0) - prev_time;
  if (integrateGyro) {
    // Kobuki
    //globalAng.x_axis += data.angular_velocity.x * time_diff * 180 / 3.1415;
    //globalAng.y_axis += data.angular_velocity.y * time_diff * 180 / 3.1415;
    // Romi
    globalAng.x_axis += data.angular_velocity.y * time_diff * 180 / 3.1415;
    globalAng.y_axis += -data.angular_velocity.x * time_diff * 180 / 3.1415;
    globalAng.z_axis += data.angular_velocity.z * time_diff * 180 / 3.1415;
  }
  runningGlobalAng.x_axis += data.angular_velocity.x * time_diff * 180 / 3.1415;
  runningGlobalAng.y_axis += data.angular_velocity.y * time_diff * 180 / 3.1415;
  runningGlobalAng.z_axis += data.angular_velocity.z * time_diff * 180 / 3.1415;
  if (monitors) {
    ROS_INFO("Angle: (x, y, z) = (%f, %f, %f)", runningGlobalAng.x_axis, runningGlobalAng.y_axis, runningGlobalAng.z_axis);
  }
  prev_time = data.header.stamp.sec + (data.header.stamp.nsec / 1000000000.0);
}

void processBump(const kobuki_msgs::BumperEvent& data) {
  if (data.state == kobuki_msgs::BumperEvent::PRESSED){
    bp = true;
  } else {
    bp = false;
  }
  if (monitors) {
    ROS_INFO("Bumper Event: %d", data.bumper);
  }
  which_bp = data.bumper;
  if (which_bp == 0) {
    newSensors.bumps_wheelDrops.bumpLeft = true;
  } else if (which_bp == 1) {
    newSensors.bumps_wheelDrops.bumpCenter = true;
  } else if (which_bp == 2) {
    newSensors.bumps_wheelDrops.bumpRight = true;
  }
  
}

void processCliff(const kobuki_msgs::CliffEvent& data) {
  if (data.state == kobuki_msgs::CliffEvent::CLIFF) {
    cf = true;
  } else {
    cf = false;
  }
  if (monitors) {
    ROS_INFO("Cliff Event: %d", data.sensor);
  }
  which_cf = data.sensor;
  cf_dis = data.bottom;
  if (which_cf == 0) {
    newSensors.cliffLeft = true;
  } else if (which_cf == 1) {
    newSensors.cliffCenter = true;
  } else if (which_cf == 2) {
    newSensors.cliffRight = true;
  }
}

void processButton(const std_msgs::Bool::ConstPtr& msg) {
  newSensors.button_pressed = true;
}

void updatePoseData(const ros::TimerEvent&) {
  for (int i = 0; i < NUM_ROBOTS; ++i) {
    pose_data[i] = recent_poses[i];
  }
}

void processModel(const gazebo_msgs::ModelStates& msg) {
  std::string pre = "Robot";
  // Not sure how to bound this
  for (int i = 0; i < NUM_ROBOTS + 1; ++i) {
    if (msg.name[i].compare(0, pre.size(), pre) == 0) {
      int num = (msg.name[i].at(5) - '0') - 1;
      recent_poses[num] = msg.pose[i];
    }
  }
}

int main(int argc, char **argv)
{
  // Instantiate node
  ros::init(argc, argv, "romi_emulator");
  ros::NodeHandle n;

  // Publishers and subscribers
  pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 20);
  ros::Subscriber sub1 = n.subscribe("odom", 1, &processOdometry);
  ros::Subscriber sub2 = n.subscribe("mobile_base/sensors/imu_data", 1, &processIMU);
  ros::Subscriber sub3 = n.subscribe("mobile_base/events/bumper", 1, &processBump);
  ros::Subscriber sub4 = n.subscribe("mobile_base/events/cliff", 1, &processCliff);
  ros::Subscriber sub5 = n.subscribe("button_press", 1000, &processButton);


  ros::Subscriber global_pos = n.subscribe("/gazebo/model_states", 1, &processModel);

  ros::Timer timer1 = n.createTimer(ros::Duration(1.0f / POSE_UPDATE_HZ), updatePoseData);
  std::string ns = ros::this_node::getNamespace();

  

  for (int i = 0; i < NUM_ROBOTS; ++i) {
    std::stringstream check;
    check << "//r" << i;

    if (!ns.compare(check.str())) {
      robot_idx = i;
    }
  }

  nrf_delay_ms(1000);

  robot_state_t state = OFF;

  // loop forever, running state machine
  while (ros::ok()) {

    // spinOnce to allow ros node to publish
    ros::spinOnce();
    nrf_delay_ms(100);

    // update state
    robot_state_t new_state = controller(state);
    state = new_state;

    // Reset sensor values (cliff and bump publishers don't publish values of false)
    newSensors.cliffLeft = false;
    newSensors.cliffCenter = false;
    newSensors.cliffRight = false;
    newSensors.bumps_wheelDrops.bumpLeft = false;
    newSensors.bumps_wheelDrops.bumpCenter = false;
    newSensors.bumps_wheelDrops.bumpRight = false;
  }

  return 0;
}
