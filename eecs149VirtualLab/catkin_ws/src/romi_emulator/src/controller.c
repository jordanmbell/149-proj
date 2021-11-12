#include "controller.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION true

#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"
#endif

// Configure initial state
KobukiSensors_t sensors = {0};

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
// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop
robot_state_t controller(robot_state_t state) {

  kobukiSensorPoll(&sensors);
  float tilt = read_tilt();

    // handle states
    switch(state) {
      case OFF: {
        printf("OFF\n");

        // transition logic
        if (is_button_pressed(&sensors)) {
          prev_encoder = sensors.leftWheelEncoder;
          state = DRIVING;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }
        break; // each case needs to end with break!
      }
      case DRIVING: {
        printf("Driving\n");

        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else {
          state = DRIVING;
          // perform state-specific actions here
          kobukiDriveDirect(100, 100);
          float dist = measure_distance(sensors.leftWheelEncoder, prev_encoder);
          prev_encoder = sensors.leftWheelEncoder;
          printf("distance %f\n", dist);
        }
        break; // each case needs to end with break!
      }

    }
    return state;
}
