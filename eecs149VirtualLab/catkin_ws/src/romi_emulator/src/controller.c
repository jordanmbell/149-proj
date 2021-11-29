#include "controller.h"

#include <math.h>
#include <stdio.h>

#define SIMULATION true
#define wheel_distance 0.15
#define NUM_ROBOTS 4
#define pi 3.141592653589793

#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "display.h"
#include "kobukiSensorTypes.h"
#endif

// Return distance traveled between two encoder values
static float get_distance(uint16_t current_encoder, uint16_t prev_encoder) {
  const float CONVERSION = 0.0006108;
  int32_t diff;
  float distance;

  if (current_encoder < prev_encoder) {
    diff = (1 << 16) + current_encoder - prev_encoder;
  } else {
    diff = current_encoder - prev_encoder;
  }

  // diff = (int32_t) current_encoder - (int32_t) prev_encoder;

  distance = diff * CONVERSION;
  return distance;
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_and_save_bump(KobukiSensors_t* sensors,
                                bool* obstacle_is_right) {
  // Your code here
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right) {
  if (sensors->cliffLeft || sensors->cliffRight || sensors->cliffCenter) {
    if (sensors->cliffRight && cliff_is_right != NULL) {
      *(cliff_is_right) = true;
    } else {
      *(cliff_is_right) = false;
    }
    return true;
  }
  return false;
}

// Read accelerometer value and calculate and return tilt (along axis
// corresponding to climbing the hill)
static float read_tilt() {
  // Your code here
  lsm9ds1_measurement_t angles = lsm9ds1_read_accelerometer();
  return atan(angles.y_axis / sqrt((angles.x_axis * angles.x_axis) +
                                   (angles.z_axis * angles.z_axis))) *
         180 / M_PI;
}

// Perform a PTP algorithm to detect clock error
static double estimate_clock_error() {
  double t_1 = waitForServerTime();
  double t_2e = currentTime();
  double t_3e;
  double t_4 = waitForServerResponse(&t_3e);

  double RTT = (t_4 - t_1) - ((t_3e) - (t_2e));
  return (t_2e) - (t_1 + RTT / 2);
}

static uint16_t new_command_length(uint16_t LOC_ORI[], uint16_t max_count) {
    // Your code here
    uint16_t changed_counter = 0;
    uint16_t i;
    for (i = 0; i < max_count; i++){
        if (LOC_ORI[i] == 1 || LOC_ORI[i] == 2)changed_counter ++;
    }
    return changed_counter*2 + max_count;
}

static uint16_t translate_command(uint16_t LOC_ORI[],float center_command[],float command[],uint16_t LOC[],float radius[],float speed_mat[],float set_speed,uint16_t max_count, float initial_location_x, float initial_location_y, float set_radius, float time_constant,float set_turn_speed, uint16_t m) {
    //translate original command into a command list with preturn/afterturn
    uint16_t i;
    uint16_t j=0;
    for (i = 0; i < max_count; i++){
    if (LOC_ORI[i] == 0){
        command[j] = center_command[i];
        LOC[j] = LOC_ORI[i];
        radius[j] = -1;
        speed_mat[j] = set_speed;
        j++;
    } else if (LOC_ORI[i] == 2){
        command[j] = atan(initial_location_y/(set_radius-initial_location_x))/pi*180;
        LOC[j] = LOC_ORI[i];
        radius[j] = 0;
        speed_mat[j] = atan(initial_location_y/(set_radius-initial_location_x))/time_constant;
        j++;

        command[j] = center_command[i];
        LOC[j] = LOC_ORI[i];
        radius[j] = set_radius;
        speed_mat[j] = set_turn_speed;
        j++;

        command[j] = atan(initial_location_y/(set_radius-initial_location_x))/pi*180;
        LOC[j] = 3-LOC_ORI[i];
        radius[j] = 0;
        speed_mat[j] = atan(initial_location_y/(set_radius-initial_location_x))/time_constant;
        j++;
    } else if (LOC_ORI[i] == 1){
        command[j] = atan(initial_location_y/(set_radius+initial_location_x))/pi*180;
        LOC[j] = LOC_ORI[i];
        radius[j] = 0;
        speed_mat[j] = atan(initial_location_y/(set_radius+initial_location_x))/time_constant;
        j++;

        command[j] = center_command[i];
        LOC[j] = LOC_ORI[i];
        radius[j] = set_radius;
        speed_mat[j] = set_turn_speed;
        j++;

        command[j] = atan(initial_location_y/(set_radius+initial_location_x))/pi*180;
        LOC[j] = 3-LOC_ORI[i];
        radius[j] = 0;
        speed_mat[j] = atan(initial_location_y/(set_radius+initial_location_x))/time_constant;
        j++;
    }
	}
    LOC[j] = 9;

    for (j = 0; j < m; j++){
		if(command[j]<0){
            command[j]= -command[j];
            LOC[j] = 3 - LOC[j];
            speed_mat[j] = -speed_mat[j];
		}
        }
    return 0;
}

static uint16_t get_relative_xy(float *relative_x, float *relative_y, int counter, uint16_t LOC[], float command[], float time, float speed, float radius, float current_x, float current_y, float initx, float inity){
    if (radius == 0){
        *relative_x = 0;
        *relative_y = 0;
        return 1;
    }

    float init_direction = 0;
    float supposed_x, supposed_y, theta;
    int i = 0;
    for (i = 0; i < counter; i++){
        if (LOC[i] == 1){
            init_direction += command[i];
        } else if (LOC[i] == 2){
            init_direction -= command[i];
        }
    }
    init_direction = init_direction/180*pi;
    //printf("initdire = %f \n",init_direction);

    if (LOC[counter] == 0){
        theta = init_direction;
        supposed_x = initx - time * speed/1000 * sin(theta);
        supposed_y = inity + time * speed/1000 * cos(theta);
    } else if (LOC[counter] == 1){
        theta = init_direction + speed/1000/radius * time;
        supposed_x = initx - radius*cos(init_direction) + radius*cos(theta);
        supposed_y = inity - radius*sin(init_direction) + radius*sin(theta);
    } else if (LOC[counter] == 2){
        theta = init_direction - speed/1000/radius * time;
        supposed_x = initx + radius*cos(init_direction) - radius*cos(theta);
        supposed_y = inity + radius*sin(init_direction) - radius*sin(theta);
    }

    *relative_x = cos(theta)*(current_x-supposed_x)+sin(theta)*(current_y-supposed_y);
    *relative_y = -sin(theta)*(current_x-supposed_x)+cos(theta)*(current_y-supposed_y);
    return 0;
}

// Configure initial state
KobukiSensors_t sensors = {0};
robot_position_t robot_positions[NUM_ROBOTS];
robot_position_t* my_position;

float xinit[] = {0.5, -0.5, 0.5, -0.5};
float yinit[] = {1, 1, -1, -1};
float initial_location_x = 0;
float initial_location_y = 0;
float current_x, current_y;
float relative_x=0, relative_y=0, velocity;
float center_command[] = {1.5, 90, 1.5, 90, 1.5};
uint16_t LOC_ORI[] = {0, 1, 0, 2, 0};  // 1 left,2 right
float set_speed = 200;
float set_turn_speed = 200;
float set_radius = 1.5;
float time_constant = 2;
float set_distance_or_angle, measure_distance_or_angle;
float current_time, enter_state_time;
float init_state_x = 0, init_state_y = 0;
uint16_t changed_counter = 0;
uint16_t initial_encoder;
uint16_t counter = 0;
robot_state_t next_state;
uint16_t m;
float rad, radd, spd;
float Kp1 = 2000;
float Kp2 = 2;
float Ki1;
float Ki2;
float Kd1;
float Kd2;
float d1,d2,i1,i2;
uint16_t max_count = sizeof(center_command) / sizeof(center_command[0]);
uint16_t j = 0;
uint16_t i = 0;
float command[20];
uint16_t LOC[20];
float radius[20];
float speed_mat[20];
double clock_error = -1;
int robot_num = -1;

static double get_server_time() { return currentTime() - clock_error; }

// You may need to add additional variables to keep track of state here
uint16_t prev_encoder = 0;
uint16_t timer = 0;
double robot_angles[NUM_ROBOTS];

double last_clock_time = 0;

// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop
robot_state_t controller(robot_state_t state) {
  kobukiSensorPoll(&sensors);
  globalPositionPoll(robot_positions);
  globalAnglesPoll(robot_angles);
  float tilt = read_tilt();
  double server_time = get_server_time();

  // handle states
  switch (state) {
    case OFF: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        printf("Starting timer\n");
        clock_error = estimate_clock_error();
        state = GETTING_NUM;
        timer = 50;
        m = new_command_length(LOC_ORI,max_count);
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
        state = PENDING;
        counter = 0;
        initial_encoder = sensors.rightWheelEncoder;
        measure_distance_or_angle = 0;
        my_position = robot_positions + robot_num;

        initial_location_x = xinit[robot_num];
        initial_location_y = yinit[robot_num];
        printf("Robot %d is at x: %f, y: %f \n", robot_num, initial_location_x,initial_location_y);
        translate_command(LOC_ORI,center_command,command,LOC,radius,speed_mat,set_speed,max_count,initial_location_x,initial_location_y,set_radius,time_constant,set_turn_speed,m);//translate original command into a command list with preturn/afterturn
        for (j = 0; j < m; j++)printf("Robot %d has com:%f, LOC:%i, RAD:%f, spd:%f \n", robot_num, command[j],  LOC[j], radius[j], speed_mat[j]);
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
          printf("Robot %d thinks it is %f, actual time is %f\n", robot_num,
                 server_time, illegal_actual_time());
          last_clock_time += 5;
        }

        state = PENDING;
        // perform state-specific actions here
        kobukiDriveDirect(0, 0);
      }
      // if(server_time >= 20)state = START;
      break;  // each case needs to end with break!
    }

    case START: {
        // transition logic
    if (counter >= m){
        state = PENDING;
        counter = 0;
    } else {
        if (LOC[counter] == 1){
            state = LEADER_TURNLEFT;
            lsm9ds1_start_gyro_integration();
            printf("1 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad,  spd);
        } else if (LOC[counter] == 0){
            state = LEADER_FORWARD;
            printf("0 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad,  spd);
        } else if (LOC[counter] == 2){
            state = LEADER_TURNRIGHT;
            lsm9ds1_start_gyro_integration();
            printf("2 is at com:%f, RAD:%f, spd:%f \n", set_distance_or_angle, rad,  spd);
        }
        next_state = START;
        measure_distance_or_angle = 0;
        set_distance_or_angle = command[counter];
        rad = radius[counter];
        spd = speed_mat[counter];
        initial_encoder = sensors.rightWheelEncoder;
        enter_state_time = server_time;
        init_state_x = xinit[robot_num] - my_position->y_pos;
        init_state_y = yinit[robot_num] + my_position->x_pos;
        counter += 1;
        i1=0; d1=0;
        i2=0; d2=0;
    }
    break; // each case needs to end with break!
    }

    case LEADER_FORWARD: {
      // variable: set_speed, set_distance_or_angle, next_state, rad, spd
      //  transition logic
        if (is_button_pressed(&sensors)) {
            state = PENDING;
        } else if (measure_distance_or_angle >= set_distance_or_angle) {
            state = next_state;
            kobukiDriveDirect(0, 0);
            measure_distance_or_angle = 0;
            initial_encoder = sensors.rightWheelEncoder;
            lsm9ds1_stop_gyro_integration();
        } else {
            display_write("LEADER_FORWARD", DISPLAY_LINE_0);
            current_x = xinit[robot_num] - my_position->y_pos;
            current_y = yinit[robot_num] + my_position->x_pos;
            get_relative_xy(&relative_x, &relative_y, counter-1, LOC, command, server_time - enter_state_time, spd, -1, current_x, current_y, init_state_x, init_state_y);
            printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f \n",current_x, current_y, init_state_x, init_state_y, relative_x, relative_y);
            //printf("t: %f \n",server_time);
            d1 = relative_x - d1;
            d2 = relative_y - d2;
            i1 += relative_x;
            i2 += relative_y;
            kobukiDriveDirect(spd - relative_y*Kp1 + d1*Kd1 + i1*Ki1, Kp2*relative_x + d2*Kd2 + i2*Ki2);
            measure_distance_or_angle = get_distance(sensors.rightWheelEncoder, initial_encoder);
            char line[16];
            snprintf(line, 16, "%f", measure_distance_or_angle);
            display_write(line, DISPLAY_LINE_1);
            printf("\n");
        }
        break;  // each case needs to end with break!
    }

    case LEADER_TURNLEFT: {
      // variable: set_speed, set_distance_or_angle, set_radius, next_state,
      // rad, spd
      //  transition logic

        if (is_button_pressed(&sensors)) {
            state = PENDING;
            lsm9ds1_stop_gyro_integration();
        } else if (measure_distance_or_angle >= set_distance_or_angle) {
            state = next_state;
            kobukiDriveDirect(0, 0);
            measure_distance_or_angle = 0;
            initial_encoder = sensors.rightWheelEncoder;
            lsm9ds1_stop_gyro_integration();
        } else {
            display_write("LEADER_TURNLEFT", DISPLAY_LINE_0);
            if (rad != 0) {
              velocity = spd / rad * (sqrt(pow(initial_location_y, 2) + pow(rad + initial_location_x, 2)));
              radd = sqrt(pow(rad + initial_location_x, 2) + pow(initial_location_y, 2));
              current_x = xinit[robot_num] - my_position->y_pos;
              current_y = yinit[robot_num] + my_position->x_pos;
              get_relative_xy(&relative_x, &relative_y, counter-1, LOC, command, server_time - enter_state_time, velocity, radd, current_x, current_y, init_state_x, init_state_y);
              printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f \n",current_x, current_y, init_state_x, init_state_y, relative_x, relative_y);
              //printf("t: %f \n",server_time);
              d1 = relative_x - d1;
              d2 = relative_y - d2;
              i1 += relative_x;
              i2 += relative_y;
              kobukiDriveDirect(velocity - relative_y*Kp1 + d1*Kd1 + i1*Ki1, velocity/radd/1000 + Kp2*relative_x + d2*Kd2 + i2*Ki2);
              lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
              measure_distance_or_angle = meas.z_axis;
              char line[16];
              snprintf(line, 16, "%f", measure_distance_or_angle);
              display_write(line, DISPLAY_LINE_1);
              printf("\n");
            } else {
              kobukiDriveDirect(0, spd);
              lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
              measure_distance_or_angle = meas.z_axis;
              char line[16];
              snprintf(line, 16, "%f", measure_distance_or_angle);
              display_write(line, DISPLAY_LINE_1);
              printf("\n");
            }
        }
      break;  // each case needs to end with break!
    }

    case LEADER_TURNRIGHT: {
      // variable: set_speed, set_distance_or_angle, set_radius, next_state,
      // rad, spd
      //  transition logic
        if (is_button_pressed(&sensors)) {
            state = PENDING;
            lsm9ds1_stop_gyro_integration();
        } else if (measure_distance_or_angle <= -set_distance_or_angle) {
            state = next_state;
            kobukiDriveDirect(0, 0);
            measure_distance_or_angle = 0;
            initial_encoder = sensors.rightWheelEncoder;
            lsm9ds1_stop_gyro_integration();
        } else {
              display_write("LEADER_TURNRIGHT", DISPLAY_LINE_0);
              if (rad != 0) {
              velocity = spd / rad * (sqrt(pow(initial_location_y, 2) + pow(rad - initial_location_x, 2)));
              radd = sqrt(pow(rad - initial_location_x, 2) + pow(initial_location_y, 2));
              current_x = xinit[robot_num] - my_position->y_pos;
              current_y = yinit[robot_num] + my_position->x_pos;
              get_relative_xy(&relative_x, &relative_y, counter-1, LOC, command, server_time - enter_state_time, velocity, radd, current_x, current_y, init_state_x, init_state_y);
              printf("x %f, y %f, inx %f, iny %f,rx %f, ry %f \n",current_x, current_y, init_state_x, init_state_y, relative_x, relative_y);
              d1 = relative_x - d1;
              d2 = relative_y - d2;
              i1 += relative_x;
              i2 += relative_y;
              kobukiDriveDirect(velocity - relative_y*Kp1 + d1*Kd1 + i1*Ki1, -velocity/radd/1000 + Kp2*relative_x + d2*Kd2 + i2*Ki2);
              lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
              measure_distance_or_angle = meas.z_axis;
              char line[16];
              snprintf(line, 16, "%f", measure_distance_or_angle);
              display_write(line, DISPLAY_LINE_1);
              printf("\n");
            } else {
              kobukiDriveDirect(0, -spd);
              lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
              measure_distance_or_angle = meas.z_axis;
              char line[16];
              snprintf(line, 16, "%f", measure_distance_or_angle);
              display_write(line, DISPLAY_LINE_1);
              printf("\n");
            }
        }
        break;  // each case needs to end with break!
        }
    }
    return state;
}
