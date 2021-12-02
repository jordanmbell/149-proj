#include <math.h>
#include <stdio.h>
#include <time.h>

#include "controller.h"
#include "kobukiSensorTypes.h"
#include "display.h"

#define NUM_ROBOTS 4

// configure initial state
KobukiSensors_t sensors = {0};
uint16_t previous_encoder = 0;
float distance = 0;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.00065;

  float result = 0.0;
  if (current_encoder >= previous_encoder) {
    result = (float)current_encoder - (float)previous_encoder;
  } else {
    result = (float)current_encoder + (0xFFFF - (float)previous_encoder);
  }
  return result = result * CONVERSION;
}

// Timer for getting the number of the robot
double num_timer; 

// Number of this robot
int robot_num = 0;
robot_state_t controller(robot_state_t state) {
  // read sensors from robot
  kobukiSensorPoll(&sensors);

  // delay before continuing
  // Note: removing this delay will make responses quicker, but will result
  //  in printf's in this loop breaking JTAG
  //nrf_delay_ms(1);

  char buf[16];

  // handle states
  switch(state) {
    case OFF: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        state = GETTING_NUM;
        num_timer = (double) time(NULL);
      } else {
        // perform state-specific actions here
        display_write("OFF", DISPLAY_LINE_0);
        kobukiDriveDirect(0,0);
        state = OFF;
      }
      break; // each case needs to end with break!
    }

    case GETTING_NUM: {
      if (is_button_pressed(&sensors)) {
        robot_num += 1;
      } else if (num_timer + 30 <= (double) time(NULL)) {
        state = PENDING;
      } else {
        snprintf(buf, 16, "%f", num_timer + 30 - (double) time(NULL));
        display_write(buf, DISPLAY_LINE_1);
        display_write("GETTING_NUM", DISPLAY_LINE_0);
        state = GETTING_NUM;
        kobukiDriveDirect(0, 0);
      }
      break;
    }

    case PENDING: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        state = START;
        counter = 0;
        initial_encoder = sensors.rightWheelEncoder;
        measure_distance_or_angle = 0;
      } else {
        // display_write("PENDING", DISPLAY_LINE_0);
        // printf("\n");

        if (server_time > last_clock_time) {
          printf("Robot %d thinks it is %f\n", robot_num, server_time);
          last_clock_time += 5;
        }

        state = PENDING;
        // perform state-specific actions here
        kobukiDriveDirect(0, 0);
      }
      if(server_time >= 20)state = START;
      break;  // each case needs to end with break!
    }
  }

  // add other cases here
  return state;
}