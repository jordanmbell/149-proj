#include "controller.h"

#include <math.h>
#include <stdio.h>

#include "app_util.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "simple_ble.h"

#define NUM_ROBOTS 4
/****** BLE SETUP ****/
typedef struct {
  double x_pos;
  double y_pos;
  double angle;
} rob_data_t;

typedef struct {
  double timestamp;
  rob_data_t robot_data[NUM_ROBOTS];
} incoming_data_t;

// BLE vars
incoming_data_t incoming_data;
rob_data_t robot_data[NUM_ROBOTS];
// configure initial state
KobukiSensors_t sensors = {0};
float distance = 0;
// Timer for getting the number of the robot
double num_timer;
// Current robot time
double current_time;
// Current error between robot and server
double current_error = 0;
// Number of this robot
int robot_num = 0;
// Buffer for string writes.
char buf[16];
// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    // c0:98:e5:49:xx:xx
    .platform_id = 0x49,      // used as 4th octect in device BLE address
    .device_id = 0x9870,      // TODO: replace with your lab bench number
    .adv_name = "EE149 LED",  // used in advertisements if there is room
    .adv_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t pos_service = {
    {.uuid128 = {0x70, 0x6C, 0x98, 0x41, 0xCE, 0x43, 0x14, 0xA9, 0xB5, 0x4D,
                 0x22, 0x2B, 0x89, 0x10, 0xE6, 0x32}}};

static simple_ble_char_t pos_state_char = {.uuid16 = 0x108a};

// Main application state
simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
  printf("Enter BLE Handle\n");
  if (simple_ble_is_char_event(p_ble_evt, &pos_state_char)) {
    printf("Got robot data!\n");
    current_time = incoming_data.timestamp;
    for (int i = 0; i < NUM_ROBOTS; i++) {
      robot_data[i].x_pos = incoming_data.robot_data[i].x_pos;
      robot_data[i].y_pos = incoming_data.robot_data[i].y_pos;
      robot_data[i].angle = incoming_data.robot_data[i].angle;
    }
  }
  printf("Exit BLE Handle\n");
}

static void setup_ble() {
  ble_config.device_id += robot_num;

  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&pos_service);

  simple_ble_add_characteristic(1, 1, 0, 0, sizeof(incoming_data),
                                (uint8_t*)&incoming_data, &pos_service,
                                &pos_state_char);

  // Start Advertising
  simple_ble_adv_only_name();
}

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

robot_state_t controller(robot_state_t state) {
  if (state != OFF && state != GETTING_NUM) {
    power_manage();
  } else {
    current_time += 0.02;
  }
  // read sensors from robot
  kobukiSensorPoll(&sensors);

  // delay before continuing
  // Note: removing this delay will make responses quicker, but will result
  //  in printf's in this loop breaking JTAG
  nrf_delay_ms(1);

  // handle states
  switch(state) {
    case OFF: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        state = GETTING_NUM;
        num_timer = current_time + 10;
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
        robot_num = (robot_num + 1) % NUM_ROBOTS;
      } else if (num_timer <= current_time) {
        setup_ble();
        state = PENDING;
      } else {
        snprintf(buf, 16, "%d, %f", robot_num, num_timer - current_time);
        printf("%d, %f\n", robot_num, current_time);
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
        state = OFF;
      } else {
        snprintf(buf, 16, "server_t: %f", current_time);
        display_write(buf, DISPLAY_LINE_1);
        display_write("PENDING", DISPLAY_LINE_0);
        state = PENDING;
        // perform state-specific actions here
        kobukiDriveDirect(0, 0);
      }
      break;  // each case needs to end with break!
    }
  }

  // add other cases here
  return state;
}