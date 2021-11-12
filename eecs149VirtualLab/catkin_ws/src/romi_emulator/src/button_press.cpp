/** Simple ROS node to publish to the "button_press" topic
*   Author: Bernard Chen
*   ----------
*   Used by romi_emulator to substitute physical button presses
*/

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>

int main(int argc, char **argv)
{
  // Init ROS node to emulate romi button press
  ros::init(argc, argv, "button_press_pub");
  ros::NodeHandle n("r1");
  ros::NodeHandle n2("r2");
  ros::NodeHandle n3("r3");
  // ros::NodeHandle n;
  ros::Publisher button_pub = n.advertise<std_msgs::Bool>("button_press", 1000);
  ros::Publisher button_pub_2 = n2.advertise<std_msgs::Bool>("button_press", 1000);
  ros::Publisher button_pub_3 = n3.advertise<std_msgs::Bool>("button_press", 1000);

  std_msgs::Bool press;

  // Check for single button press argument
  // Send single button press and exit
  if (argc == 2) {
    // std::string arg = argv[1];
    // if (arg.compare("-s") || arg.compare("--single")) {
    //   press.data = true;

    //   // Wait for subscriber before publishing once
    //   ros::Rate poll_rate(100);
    //   while(button_pub.getNumSubscribers() == 0)
    //     poll_rate.sleep();

    //   button_pub.publish(press);
    //   exit(0);
    // }
    std::cout << "Silent mode not supported, please run without \'-s\'";
    exit(0);
  }

  // Otherwise take user input
  while (ros::ok())
  {
    std::cout << "Press 1-3 to send a Kobuki button press to the corresponding robot\n";

    // Wait for 'Enter' key press
    char robot_num;
    std::cin >> robot_num;
    if (robot_num == '1') {
      press.data = true;
      button_pub.publish(press);
    } else if (robot_num == '2') {
      press.data = true;
      button_pub_2.publish(press);
    } else if (robot_num == '3') {
      press.data = true;
      button_pub_3.publish(press);
    } else {
      std::cout << "Invalid key";
    }
  }

  return 0;
}