#include <stdint.h>

typedef enum {
  OFF,
  DRIVING,
  GETTING_NUM,
  PENDING,
  DISPLAY_TIME,
  DISPLAY_X,
  DISPLAY_Y,
  DISPLAY_ANG,
} robot_state_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);

robot_state_t controller(robot_state_t state);