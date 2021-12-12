#include "controller.h"

#include <math.h>
#include <stdio.h>

#include "app_util.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "simple_ble.h"

#define wheel_distance 0.229
#define NUM_ROBOTS 4
#define MAX_COMMANDS 8
#define IN_PLACE_SPEED 1.5

/****** BLE SETUP ****/
typedef struct
{
  double x_pos;
  double y_pos;
  double angle;
} rob_data_t;

typedef struct
{
  double timestamp;
  rob_data_t robot_data[NUM_ROBOTS];
  int cmd_len;
  int trace_cmd[MAX_COMMANDS];
  double trace_dist_angle[MAX_COMMANDS];
  double trace_time[MAX_COMMANDS];
  double start_time;
} incoming_data_t;

// BLE vars
incoming_data_t incoming_data;
rob_data_t robot_data[NUM_ROBOTS] = {0};
double start_time = 5;
bool connected = false;
bool updated_data = false;
double server_time;
// configure initial state
double distance = 0;
// Timer for getting the number of the robot
double num_timer;
// Current robot time
double current_time;
double time_incr = 0.02;
// Buffer for string writes.
char buf[16];
// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    // c0:98:e5:49:xx:xx
    .platform_id = 0x49,     // used as 4th octect in device BLE address
    .device_id = 0x9870,     // TODO: replace with your lab bench number
    .adv_name = "EE149 LED", // used in advertisements if there is room
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
simple_ble_app_t *simple_ble_app;

// Configure initial state
KobukiSensors_t sensors = {0};
rob_data_t robot_positions[NUM_ROBOTS];
double initial_location_x = 0.1;
double initial_location_y = 0.1;
double current_x = 0.1;
double current_y = 0.1;
double current_ang = 0;
double relative_x = 0, relative_y = 0, velocity;
int encoder_at_last_measure;
int command_idx = 0;
double cur_distance_from_end;
double command_length[MAX_COMMANDS] = {10, 12, 10, 12, 10, 12, 10, 12}; // {2, 6, 2, 2, 6, 2, 10, 2, 6, 2, 10}
double center_command[MAX_COMMANDS] = {0.75, 90, 0.75, 90, 0.75, 90, 0.75, 90};
int LOC_ORI[MAX_COMMANDS] = {0, 1, 0, 1, 0, 1, 0, 1}; // 1 left,2 right
double set_radius[MAX_COMMANDS] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
uint16_t max_count = 8;
double time_constant = 2;
double set_distance_or_angle, measure_distance_or_angle;
double enter_state_time;
double init_state_x = 0.1, init_state_y = 0.1;
uint16_t initial_encoder;
double initial_angle;
uint16_t counter = 0;
robot_state_t next_state;
uint16_t m;
double rad, radd, spd;
double Kp1 = 2000;
double Kp2 = 2;
double Ki1;
double Ki2;
double Kd1;
double Kd2;
double d1, d2, i1, i2;
double command[MAX_COMMANDS * 3];
uint16_t LOC[MAX_COMMANDS * 3];
double radius[MAX_COMMANDS * 3];
double speed_mat[MAX_COMMANDS * 3];
double modified_r_mat[MAX_COMMANDS * 3];
double LOC_TIME[MAX_COMMANDS * 3];
uint16_t last_right;
uint16_t last_left;
int robot_num = 0;
bool turning_in_place = false;
double last_global_angle = 0;
double update_trust = 0.1;

// You may need to add additional variables to keep track of state here
double timer = 0;

double last_clock_time = 0;

void ble_evt_write(ble_evt_t const *p_ble_evt)
{
  printf("Enter BLE Handle\n");
  if (simple_ble_is_char_event(p_ble_evt, &pos_state_char))
  {
    if (!connected) {
      // Parse trace data
      start_time = incoming_data.start_time;
      max_count = incoming_data.cmd_len;
      for (int k = 0; k < incoming_data.cmd_len; k++) {
        LOC_ORI[k] = incoming_data.trace_cmd[k];
        command_length[k] = incoming_data.trace_time[k];
        center_command[k] = incoming_data.trace_dist_angle[k];
        set_radius[k] = 0.3;
      }
      
    }
    updated_data = true;
    printf("Got robot data!\n");
    server_time = incoming_data.timestamp;
    for (int k = 0; k < NUM_ROBOTS; k++)
    {
      robot_data[k].x_pos = incoming_data.robot_data[k].x_pos;
      robot_data[k].y_pos = incoming_data.robot_data[k].y_pos;
      robot_data[k].angle = incoming_data.robot_data[k].angle;
    }
  }
  printf("Exit BLE Handle\n");
}

static void setup_ble()
{
  ble_config.device_id += robot_num;

  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&pos_service);

  simple_ble_add_characteristic(1, 1, 0, 0, sizeof(incoming_data),
                                (uint8_t *)&incoming_data, &pos_service,
                                &pos_state_char);

  // Start Advertising
  simple_ble_adv_only_name();
}

static double get_distance(uint16_t current_encoder, uint16_t prev_encoder) {
  const double CONVERSION = 0.0000853;

  double result = 0.0;
  if (current_encoder >= prev_encoder) {
    result = (double)current_encoder - (double)prev_encoder;
  } else if (current_encoder < 0xFFFF/2 && prev_encoder > 0xFFFF/2) {
    result = (double)current_encoder + (0xFFFF - (double)prev_encoder);
  } else {
    // BACKWARDS
    result = 0;
  }
  return result = result * CONVERSION;
}

static uint16_t new_command_length()
{
  // Your code here
  uint16_t changed_counter = 0;
  uint16_t k;
  for (k = 0; k < max_count; k++)
  {
    if (LOC_ORI[k] == 1 || LOC_ORI[k] == 2)
      changed_counter++;
  }
  return changed_counter * 2 + max_count;
}

static uint16_t translate_command()
{
  // translate original command into a command list with preturn/afterturn
  uint16_t i;
  uint16_t j = 0;
  for (i = 0; i < max_count; i++)
  {
    if (LOC_ORI[i] == 0)
    {
      command[j] = center_command[i];
      LOC[j] = LOC_ORI[i];
      radius[j] = -1;
      speed_mat[j] = center_command[i]/command_length[i]*1000;
      LOC_TIME[j] = command_length[i];
      j++;
    }
    else if (LOC_ORI[i] == 2)
    {
      command[j] =
          atan(initial_location_y / (set_radius[i] - initial_location_x)) /
          M_PI * 180;
      LOC[j] = LOC_ORI[i];
      radius[j] = 0;
      speed_mat[j] = atan(initial_location_y / (set_radius[i] - initial_location_x)) / time_constant;
      LOC_TIME[j] = time_constant;
      j++;

      command[j] = center_command[i];
      LOC[j] = LOC_ORI[i];
      radius[j] = set_radius[i];
      speed_mat[j] = center_command[i]*set_radius[i]/180*M_PI/(command_length[i]-2*time_constant)*1000;
      LOC_TIME[j] = command_length[i]-2*time_constant;
      j++;

      command[j] =
          atan(initial_location_y / (set_radius[i] - initial_location_x)) /
          M_PI * 180;
      LOC[j] = 3 - LOC_ORI[i];
      radius[j] = 0;
      speed_mat[j] = atan(initial_location_y / (set_radius[i] - initial_location_x)) / time_constant;
      LOC_TIME[j] = time_constant;
      j++;
    }
    else if (LOC_ORI[i] == 1)
    {
      command[j] =
          atan(initial_location_y / (set_radius[i] + initial_location_x)) /
          M_PI * 180;
      LOC[j] = LOC_ORI[i];
      radius[j] = 0;
      speed_mat[j] = atan(initial_location_y / (set_radius[i] + initial_location_x)) / time_constant;
      LOC_TIME[j] = time_constant;
      j++;

      command[j] = center_command[i];
      LOC[j] = LOC_ORI[i];
      radius[j] = set_radius[i];
      speed_mat[j] = center_command[i]*set_radius[i]/180*M_PI/(command_length[i]-2*time_constant)*1000;
      LOC_TIME[j] = command_length[i]-2*time_constant;
      j++;

      command[j] =
          atan(initial_location_y / (set_radius[i] + initial_location_x)) /
          M_PI * 180;
      LOC[j] = 3 - LOC_ORI[i];
      radius[j] = 0;
      speed_mat[j] = atan(initial_location_y / (set_radius[i] + initial_location_x)) / time_constant;
      LOC_TIME[j] = time_constant;
      j++;
    }
  }
  LOC[j] = 9;

  for (j = 0; j < m; j++)
  {
    if (command[j] < 0)
    {
      command[j] = -command[j];
      LOC[j] = 3 - LOC[j];
      speed_mat[j] = -speed_mat[j];
    }
  }
  return 0;
}

static double get_relative_xy(double time, double speed, double radius_cur)
{
    if (FP_ZERO == fpclassify(radius_cur))
    {
        relative_x = 0;
        relative_y = 0;
        return 1;
    }

    double init_direction = 0;
    double initx = initial_location_x;
    double inity = initial_location_x;
    double supposed_x, supposed_y, theta;
    uint16_t i = 0;
    for (i = 0; i < counter - 1; i++)
    {
        if (FP_ZERO != fpclassify(modified_r_mat[i])) // check if radius 0
        {
            if (LOC[i] == 0)
            {
                initx -= command[i] * sin(init_direction);
                inity += command[i] * cos(init_direction);
            }
            else if (LOC[i] == 1)
            {
                initx = initx - modified_r_mat[i] * cos(init_direction) + modified_r_mat[i] * cos(init_direction + command[i] / 180 * M_PI);
                inity = inity - modified_r_mat[i] * sin(init_direction) + modified_r_mat[i] * sin(init_direction + command[i] / 180 * M_PI);
            }
            else if (LOC[i] == 2)
            {
                initx = initx + modified_r_mat[i] * cos(init_direction) - modified_r_mat[i] * cos(-init_direction + command[i] / 180 * M_PI);
                inity = inity + modified_r_mat[i] * sin(init_direction) + modified_r_mat[i] * sin(-init_direction + command[i] / 180 * M_PI);
            }
        }

        if (LOC[i] == 1)
        {
            init_direction += command[i] / 180 * M_PI;
        }
        else if (LOC[i] == 2)
        {
            init_direction -= command[i] / 180 * M_PI;
        }
    }
    printf("initx = %f, inity = %f, init_tdirr = %f \n",initx, inity, init_direction);

    if (LOC[counter - 1] == 0)
    {
        theta = init_direction;
        supposed_x = initx - time * speed / 1000 * sin(theta);
        supposed_y = inity + time * speed / 1000 * cos(theta);
    }
    else if (LOC[counter - 1] == 1)
    {
        theta = init_direction + speed / 1000 / radius_cur * time;
        supposed_x = initx - radius_cur * cos(init_direction) + radius_cur * cos(theta);
        supposed_y = inity - radius_cur * sin(init_direction) + radius_cur * sin(theta);
    }
    // else if (LOC[counter - 1] == 2)
    else 
    {
        theta = init_direction - speed / 1000 / radius_cur * time;
        supposed_x = initx + radius_cur * cos(init_direction) - radius_cur * cos(theta);
        supposed_y = inity + radius_cur * sin(init_direction) - radius_cur * sin(theta);
    }
    printf("supposed_x = %f, supposed_y = %f, theta = %f, LOC = %d \n", supposed_x, supposed_y, theta, LOC[counter - 1]);

    relative_x = cos(theta) * (current_x - supposed_x) + sin(theta) * (current_y - supposed_y);
    relative_y = -sin(theta) * (current_x - supposed_x) + cos(theta) * (current_y - supposed_y);
    return theta;
}

double old_left = 0;
double old_right = 0;
static void drive_formatted(double overall_speed, double angular_speed) {
  double leftSpeed, rightSpeed;

  rightSpeed = overall_speed + wheel_distance/2*angular_speed*1000;
  leftSpeed = overall_speed - wheel_distance/2*angular_speed*1000;
  kobukiDriveDirect(leftSpeed, rightSpeed);
}

robot_state_t controller(robot_state_t state) {
  if (connected) {
    power_manage();
  }
  // read sensors from robot
  kobukiSensorPoll(&sensors);

  // delay before continuing
  // Note: removing this delay will make responses quicker, but will result
  //  in printf's in this loop breaking JTAG
  nrf_delay_ms(1);

  current_time += time_incr;

  if (updated_data) {
    updated_data = false;
    if (connected) {
      if (server_time > current_time) {
       time_incr += 0.0001;
      } else {
        time_incr -= 0.0001;
      }
    }
    current_time = server_time;
    connected = true;
    if (turning_in_place) {
      lsm9ds1_stop_gyro_integration();
      double delta_ang = atan2(sin(current_ang - robot_data[robot_num].angle),
                               cos(current_ang - robot_data[robot_num].angle));
      current_ang -= delta_ang * update_trust;
      last_global_angle = current_ang;
      lsm9ds1_start_gyro_integration();
    } else {
      double delta_x = current_x - robot_data[robot_num].x_pos;
      current_x -= delta_x * update_trust;
      double delta_y = current_y - robot_data[robot_num].y_pos;
      current_y -= delta_y * update_trust;
      double delta_ang = atan2(sin(current_ang - robot_data[robot_num].angle),
                               cos(current_ang - robot_data[robot_num].angle));
      current_ang -= delta_ang * update_trust;
      printf("cur_x: %f, cur_y: %f, cur_amg: %f\n", current_x, current_y, current_ang);
    }
  } else if (!connected && state != OFF && state != GETTING_NUM && state != PENDING && !turning_in_place) {
    double l_2 = get_distance(sensors.rightWheelEncoder, last_right);
    double l_1 = get_distance(sensors.leftWheelEncoder, last_left);

    if (l_2 > l_1) {
      // TURNING LEFT
      double theta_cur = (l_2 - l_1) / wheel_distance;
      double radius_cur = (l_2 + l_1) / (2 * theta_cur);
      current_x = current_x - radius_cur * cos(current_ang) + radius_cur * cos(current_ang + theta_cur);
      current_y = current_y - radius_cur * sin(current_ang) + radius_cur * sin(current_ang + theta_cur);
      current_ang = current_ang + theta_cur;
    } else if (l_1 > l_2) {
      // TURNING RIGHT
      double theta_cur = (l_2 - l_1) / wheel_distance;
      double radius_cur = (l_2 + l_1) / (2 * fabs(theta_cur));
      current_x = current_x + radius_cur * cos(current_ang) - radius_cur * cos(current_ang + theta_cur);
      current_y = current_y + radius_cur * sin(current_ang) - radius_cur * sin(current_ang + theta_cur);
      current_ang = current_ang + theta_cur;
    } else {
      // Went straight
      current_x -= l_1 * sin(current_ang);
      current_y += l_1 * cos(current_ang);
    }
  }
  last_right = sensors.rightWheelEncoder;
  last_left = sensors.leftWheelEncoder;

  // handle states
  switch (state) {
    case OFF: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        printf("Starting timer\n");
        state = GETTING_NUM;
        num_timer = current_time + 10;
      } else {
        // perform state-specific actions here
        display_write("OFF", DISPLAY_LINE_0);
        kobukiDriveDirect(0,0);
        state = OFF;
        if (current_time > timer + 5) {
          timer = current_time;
          snprintf(buf, 16, "%f", current_time);
          display_write(buf, DISPLAY_LINE_1);
        }
      }
      break; // each case needs to end with break!
    }

    case GETTING_NUM: {
      if (is_button_pressed(&sensors)) {
        robot_num = (robot_num + 1) % NUM_ROBOTS;
      } else if (num_timer <= current_time) {
        lsm9ds1_start_gyro_integration();
        setup_ble();
        state = PENDING;
        counter = 0;
        initial_encoder = sensors.rightWheelEncoder;
        measure_distance_or_angle = 0;
      } else {
        snprintf(buf, 16, "%d, %f", robot_num, num_timer - current_time);
        display_write(buf, DISPLAY_LINE_1);
        display_write("GETTING_NUM", DISPLAY_LINE_0);
        state = GETTING_NUM;
        drive_formatted(0, 0);
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
        drive_formatted(0, 0);
      }
      if (!connected && current_time >= start_time) {
        initial_location_x = current_x;
        initial_location_y = current_y;
        init_state_x = initial_location_x;
        init_state_y = initial_location_y;
        m = new_command_length();
        translate_command(); // translate original command into a command list with preturn/afterturn
        for (int i = 0; i < m; i++)
            {
                if (radius[i] == 0 || radius[i] == -1)
                {
                    modified_r_mat[i] = radius[i];
                }
                else if (LOC[i] == 1)
                {
                    modified_r_mat[i] = sqrt(pow(radius[i] + initial_location_x, 2) + pow(initial_location_y, 2));
                }
                else if (LOC[i] == 2)
                {
                    modified_r_mat[i] = sqrt(pow(radius[i] - initial_location_x, 2) + pow(initial_location_y, 2));
                }
            }
        for (int j = 0; j < m; j++)
          printf("Robot %d has com:%f, LOC:%i, RAD:%f, spd:%f, LOCT: %f \n", robot_num, command[j], LOC[j], radius[j], speed_mat[j], LOC_TIME[j]);
        state = START;
        command_idx = 0;
      }
      break; // each case needs to end with break!
    }

    case START:
    {
      // transition logic
      if (counter >= m)
      {
        state = OFF;
        counter = 0;
      }
      else
      {
        // Wait to stop
        if (timer > current_time) {
          drive_formatted(0, 0);
          break;
        }
        if (LOC[counter] == 1)
        {
          state = LEADER_TURNLEFT;
          printf("1 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad, spd);
          lsm9ds1_start_gyro_integration();
        }
        else if (LOC[counter] == 0)
        {
          state = LEADER_FORWARD;
          printf("0 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad, spd);
        }
        else if (LOC[counter] == 2)
        {
          state = LEADER_TURNRIGHT;
          printf("2 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad, spd);
          lsm9ds1_start_gyro_integration();
        }
        
        next_state = START;
        measure_distance_or_angle = 0;
        set_distance_or_angle = command[counter];
        rad = radius[counter];
        spd = speed_mat[counter];
        initial_encoder = sensors.rightWheelEncoder;
        initial_angle = current_ang;
        last_global_angle = initial_angle;
        enter_state_time = current_time;
        init_state_x = current_x;
        init_state_y = current_y;
        counter += 1;
        i1 = 0;
        d1 = 0;
        i2 = 0;
        d2 = 0;

      }
      break; // each case needs to end with break!
    }

    case LEADER_FORWARD:
    {
      // variable: set_speed, set_distance_or_angle, next_state, rad, spd
      //  transition logic
      if (is_button_pressed(&sensors))
      {
        state = PENDING;
      }
      else if (current_time - enter_state_time >= LOC_TIME[counter - 1])
      {
        command_idx += 1;
        state = next_state;
        drive_formatted(0, 0);
        measure_distance_or_angle = 0;
        initial_encoder = sensors.rightWheelEncoder;
        lsm9ds1_stop_gyro_integration();
        timer = current_time + 1;
      }
      else
      {
        float rel_angle = get_relative_xy(current_time - enter_state_time, spd, -1);
        // printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f \n", current_x, current_y, init_state_x, init_state_y, relative_x, relative_y);
        // printf("t: %f \n",current_time);
        d1 = relative_y - d1;
        d2 = relative_x - d2;
        i1 += relative_y;
        i2 += relative_x;
        drive_formatted(spd - relative_y * Kp1 + d1 * Kd1 + i1 * Ki1, Kp2 * relative_x + d2 * Kd2 + i2 * Ki2);
        measure_distance_or_angle = get_distance(sensors.rightWheelEncoder, initial_encoder);
        snprintf(buf, 16, "%.2f %.2f %.2f", relative_x, relative_y, rel_angle);
        display_write(buf, DISPLAY_LINE_0);
        snprintf(buf, 16, "%.2f %.2f %.2f", current_x, current_y, current_ang);
        display_write(buf, DISPLAY_LINE_1);
        // printf("encoder: %d, prev: %d, distance: %f\n", sensors.rightWheelEncoder, initial_encoder, measure_distance_or_angle);
      }
      break; // each case needs to end with break!
    }

    case LEADER_TURNLEFT:
    {
      // variable: set_speed, set_distance_or_angle, set_radius, next_state,
      // rad, spd
      //  transition logic

      if (is_button_pressed(&sensors))
      {
        state = PENDING;
        lsm9ds1_stop_gyro_integration();
      }
      else if (current_time - enter_state_time >= LOC_TIME[counter - 1])
      {
        state = next_state;
        drive_formatted(0, 0);
        measure_distance_or_angle = 0;
        initial_encoder = sensors.rightWheelEncoder;
        if (turning_in_place) {
          lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
          current_ang = meas.z_axis * M_PI / 180 + last_global_angle;
          turning_in_place = false;
        }
        
        lsm9ds1_stop_gyro_integration();
        timer = current_time + 1;
      }
      else
      {

        if (FP_ZERO != fpclassify(rad))
        {
          velocity = spd / rad * (sqrt(pow(initial_location_y, 2) + pow(rad + initial_location_x, 2)));
          radd = sqrt(pow(rad + initial_location_x, 2) + pow(initial_location_y, 2));
          double should_angle = get_relative_xy(current_time - enter_state_time, velocity, radd);
          printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f, cur_time %f, enter_time %f \n", current_x, current_y, init_state_x, init_state_y, relative_x, relative_y, current_time, enter_state_time);
          // printf("t: %f \n",current_time);
          d1 = relative_y - d1;
          d2 = relative_x - d2;
          i1 += relative_y;
          i2 += relative_x;
          drive_formatted(velocity - relative_y * Kp1 + d1 * Kd1 + i1 * Ki1, velocity / radd / 1000 + Kp2 * relative_x + d2 * Kd2 + i2 * Ki2);
          snprintf(buf, 16, "%f", current_ang);
          display_write(buf, DISPLAY_LINE_1);
          snprintf(buf, 16, "%f", relative_y);
          display_write(buf, DISPLAY_LINE_0);
        }
        else
        {
          turning_in_place = true;
          lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
          double comp_1 = fabs(meas.z_axis * M_PI / 180) +
                          fabs(last_global_angle - initial_angle);
          double comp_2 = set_distance_or_angle * M_PI / 180;
          double delta_ang = atan2(sin(comp_1 - comp_2), cos(comp_1 - comp_2));
          printf("Delta ang: %f\n", delta_ang);
          if (delta_ang <= 0) {
            // double ideal_speed = set_distance_or_angle / 180 * M_PI / time_constant;
            drive_formatted(0, IN_PLACE_SPEED);
            // printf("ideal_speed: %f, set_angle: %f, initial_angle: %f, current_angle: %f, task_time: %f, enter_time: %f, current_time: %f\n", ideal_speed, set_distance_or_angle, initial_angle, current_ang, LOC_TIME[counter - 1], enter_state_time, current_time);
            
            snprintf(buf, 16, "%f", current_ang);
            display_write(buf, DISPLAY_LINE_1);
            snprintf(buf, 16, "angle: %f", meas.z_axis * M_PI / 180 + current_ang);
            display_write(buf, DISPLAY_LINE_0);
          } else {
            drive_formatted(0, 0);
          }
        }
      }
      break; // each case needs to end with break!
    }

    case LEADER_TURNRIGHT:
    {
      // variable: set_speed, set_distance_or_angle, set_radius, next_state,
      // rad, spd
      //  transition logic
      if (is_button_pressed(&sensors))
      {
        state = PENDING;
        lsm9ds1_stop_gyro_integration();
      }
      else if (current_time - enter_state_time >= LOC_TIME[counter - 1])
      {
        state = next_state;
        drive_formatted(0, 0);
        measure_distance_or_angle = 0;
        initial_encoder = sensors.rightWheelEncoder;
        if (turning_in_place) {
          lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
          current_ang = meas.z_axis * M_PI / 180 + last_global_angle;
          turning_in_place = false;
        }
        lsm9ds1_stop_gyro_integration();
        timer = current_time + 1;
      }
      else
      {
        if (FP_ZERO != fpclassify(rad))
        {
          velocity = spd / rad * (sqrt(pow(initial_location_y, 2) + pow(rad - initial_location_x, 2)));
          radd = sqrt(pow(rad - initial_location_x, 2) + pow(initial_location_y, 2));
          get_relative_xy(current_time - enter_state_time, velocity, radd);
          // printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f \n", current_x, current_y, init_state_x, init_state_y, relative_x, relative_y);
          d1 = relative_y - d1;
          d2 = relative_x - d2;
          i1 += relative_y;
          i2 += relative_x;
          drive_formatted(velocity - relative_y * Kp1 + d1 * Kd1 + i1 * Ki1, -velocity / radd / 1000 + Kp2 * relative_x + d2 * Kd2 + i2 * Ki2);
          snprintf(buf, 16, "%f", current_ang);
          display_write(buf, DISPLAY_LINE_1);
          snprintf(buf, 16, "%f", current_time);
          display_write(buf, DISPLAY_LINE_0);
        }
        else
        {
          turning_in_place = true;
          lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
          double comp_1 = fabs(meas.z_axis * M_PI / 180) +
                          fabs(last_global_angle - initial_angle);
          double comp_2 = set_distance_or_angle * M_PI / 180;
          double delta_ang = atan2(sin(comp_1 - comp_2), cos(comp_1 - comp_2));
          if (delta_ang <= 0) {
            // double ideal_speed = set_distance_or_angle / 180 * M_PI / time_constant;
            drive_formatted(0, -1*IN_PLACE_SPEED);
            // printf("ideal_speed: %f, set_angle: %f, initial_angle: %f, current_angle: %f, task_time: %f, enter_time: %f, current_time: %f\n", ideal_speed, set_distance_or_angle, initial_angle, current_ang, LOC_TIME[counter - 1], enter_state_time, current_time);
            
            snprintf(buf, 16, "%f", set_distance_or_angle * M_PI / 180);
            display_write(buf, DISPLAY_LINE_1);
            snprintf(buf, 16, "angle: %f", meas.z_axis * M_PI / 180 + current_ang);
            display_write(buf, DISPLAY_LINE_0);
          } else {
            drive_formatted(0, 0);
          }
        }
      }
      break; // each case needs to end with break!
    }
  }
  return state;
}