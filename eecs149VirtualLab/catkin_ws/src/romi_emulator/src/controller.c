#include "controller.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION true
#define NUM_ROBOTS 3
#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"

// Includes from BLE lab
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"

#include "simple_ble.h"
#include "buckler.h"

#include "max44009.h"
#endif

// Intervals for advertising and connections
// static simple_ble_config_t ble_config = {
//         // c0:98:e5:49:xx:xx
//         .platform_id       = 0x49,    // used as 4th octect in device BLE address
//         .device_id         = 0x0012, // TODO: replace with something unique
//         .adv_name          = "EE149 LED", // used in advertisements if there is room
//         .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
//         .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
//         .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
// };

// 32e61089-2b22-4db5-a914-43ce41986c70 Maybe replace this
// static simple_ble_service_t position_service = {{
//     .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
//                 0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
// }};

// static simple_ble_char_t position_char = {.uuid16 = 0x108a};
// static robot_position_t input_data[NUM_ROBOTS];

// Main application state
// simple_ble_app_t* simple_ble_app;
// static bool updated_data = false;



// void ble_evt_write(ble_evt_t const* p_ble_evt) {
//   if (simple_ble_is_char_event(p_ble_evt, &position_char)) {
//     printf("Got updated position data!\n");
//     for (int i = 0; i < NUM_ROBOTS; i++) {
//       robot_positions[i] = input_data[i];
//     }
//     updated_data = true;
//   }
// }

// Configure initial state
KobukiSensors_t sensors = {0};
robot_position_t robot_positions[NUM_ROBOTS];
robot_position_t* my_position;

// You may need to add additional variables to keep track of state here

// Return distance traveled between two encoder values
static float measure_distance(uint16_t curr_encoder, uint16_t prev_encoder) {
  const float CONVERSION = 0.0006108;

  // Your code here
  if (curr_encoder >= prev_encoder) {
    return (curr_encoder - prev_encoder) * CONVERSION;
  } else {
    return ((0xFFFF - prev_encoder) + curr_encoder) * CONVERSION;
  }
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right) {
  // Your code here
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right) {
  // Your code here
}

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
static float read_tilt() {
  // Your code here
}



uint16_t prev_encoder = 0;
int robot_num = -1;

int timer = 0;
// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop
robot_state_t controller(robot_state_t state) {

  kobukiSensorPoll(&sensors);
  kobukiPositionPoll(robot_positions);
  float tilt = read_tilt();

    // handle states
    switch(state) {

      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          printf("Starting timer\n");
          state = GETTING_NUM;
          timer = 200;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }
        break;
      }
      case GETTING_NUM: {
        if (is_button_pressed(&sensors)) {
          robot_num += 1;
        } else if (timer <= 0) {
          state = DRIVING;
          prev_encoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(100, 100);
          my_position = robot_positions + robot_num;
        } else {
          timer -= 1;
          if (timer % 10 == 0) {
            printf("timer is: %d\n", timer);
          }
          state = GETTING_NUM;
          kobukiDriveDirect(0, 0);
        }
        break;
      }
      case DRIVING: {
        printf("Robot %d is Driving\n", robot_num);
        printf("Robot %d is at x: %f, y: %f, z: %f\n", robot_num, my_position->x_pos, my_position->y_pos, my_position->z_pos);
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = STOP;
          kobukiDriveDirect(0, 0);
        } else {
          state = DRIVING;
          kobukiDriveDirect(100, 100);
          // float dist = measure_distance(sensors.leftWheelEncoder, prev_encoder);
          // prev_encoder = sensors.leftWheelEncoder;
          // printf("distance %f\n", dist);
        }
        break; // each case needs to end with break!
      }
      case STOP: {
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
          kobukiDriveDirect(100, 100);
        } else {
          state = STOP;
          kobukiDriveDirect(0, 0);
        }
      }

    }
    return state;
}
