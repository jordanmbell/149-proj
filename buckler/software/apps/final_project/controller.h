#include <stdint.h>
#include "kobukiSensorTypes.h"

// Robot states
// Add your own states here
typedef enum {
  OFF,
  START,
  OBSTACLE,
  LEADER_FORWARD,
  LEADER_TURNLEFT,
  LEADER_TURNRIGHT,
  GETTING_NUM,
  PENDING,
} robot_state_t;

static double get_distance(uint16_t current_encoder, uint16_t prev_encoder);

robot_state_t controller(robot_state_t state);
