#include "controller.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION true
#define POSE_UPDATE_HZ 10
#define wheel_distance 0.15
#define NUM_ROBOTS 4

#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"
#endif

// Configure initial state
KobukiSensors_t sensors = {0};
robot_position_t robot_positions[NUM_ROBOTS];
robot_position_t* my_position;


float relative_location_x = 0;
float relative_location_y = 0;
float v1;
float v2;
float center_command[] = {1.5,90,1.5,90,1.5};
uint16_t LOC_ORI[] = {0,1,0,2,0};//1 left,2 right
uint16_t max_count = sizeof(center_command)/sizeof(center_command[0]);
float set_speed = 100;
float set_turn_speed = 100;
float set_radius = 1.5;
float time_constant = 2;
float set_distance_or_angle;
float measure_distance_or_angle;
uint16_t changed_counter = 0;
uint16_t initial_encoder;
uint16_t counter = 0;
robot_state_t next_state;
float rad;
float spd;

// You may need to add additional variables to keep track of state here

// Return distance traveled between two encoder values
static float get_distance(uint16_t current_encoder, uint16_t prev_encoder) {
    const float CONVERSION = 0.0006108;
    int32_t diff;
    float distance;    

    if (current_encoder < prev_encoder) {
        diff = (1 << 16) + current_encoder - prev_encoder;
    }
    else {
        diff = current_encoder - prev_encoder;
    }

    // diff = (int32_t) current_encoder - (int32_t) prev_encoder;

    distance = diff * CONVERSION;
    return distance;
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right) {
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

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
static float read_tilt() {
    // Your code here
    lsm9ds1_measurement_t angles = lsm9ds1_read_accelerometer();
    return atan(angles.y_axis / sqrt((angles.x_axis * angles.x_axis) + (angles.z_axis * angles.z_axis))) * 180 / M_PI;
}

uint16_t prev_encoder = 0;
int robot_num = -1;
int timer = 0;

// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop






robot_state_t controller(robot_state_t state) {

    uint16_t i = 0;
    for (i = 0; i < max_count; i++){
    if (LOC_ORI[i] == 1 || LOC_ORI[i] == 2){
        changed_counter ++;
    }
    }

    float command[changed_counter*2 + max_count];
    uint16_t LOC[changed_counter*2 + max_count];
    float radius[changed_counter*2 + max_count];
    float speed_mat[changed_counter*2 + max_count];

    uint16_t j = 0;


    for (j = 0; j < (changed_counter*2 + max_count); j++)
    {
        if(command[j]<0){
            command[j]= -command[j];
            LOC[j] = 3 - LOC[j];
            speed_mat[j] = -speed_mat[j];
        }
    }
    uint16_t m = changed_counter+ changed_counter + max_count;


    kobukiSensorPoll(&sensors);
    kobukiPositionPoll(robot_positions);
    float tilt = read_tilt();

    // handle states
    switch(state) {

    case STOP: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          printf("Starting timer\n");
          state = GETTING_NUM;
          timer = 50;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0, -1);
        }
        break;
      }

      case GETTING_NUM: {
        if (is_button_pressed(&sensors)) {
          robot_num += 1;
        } else if (timer <= 0) {
          state = START;
          counter = 0;
          initial_encoder = sensors.rightWheelEncoder;
          measure_distance_or_angle = 0;
          my_position = robot_positions + robot_num;
	  relative_location_x = -my_position->y_pos;
	  relative_location_y = my_position->x_pos;
	for (i = 0; i < max_count; i++)
	    {
		if (LOC_ORI[i] == 0){
		    command[j] = center_command[i];
		    LOC[j] = LOC_ORI[i];
		    radius[j] = -1;
		    speed_mat[j] = set_speed;
		    j++;
		} else if (LOC_ORI[i] == 2){
		    command[j] = atan(relative_location_y/(set_radius-relative_location_x))/3.14159265358979328626*180;
		    LOC[j] = LOC_ORI[i];
		    radius[j] = 0;
		    speed_mat[j] = atan(relative_location_y/(set_radius-relative_location_x))/time_constant/2*wheel_distance;
		    j++;

		    command[j] = center_command[i];
		    LOC[j] = LOC_ORI[i];
		    radius[j] = set_radius;
		    speed_mat[j] = set_turn_speed;
		    j++;

		    command[j] = atan(relative_location_y/(set_radius-relative_location_x))/3.14159265358979328626*180;
		    LOC[j] = 3-LOC_ORI[i];
		    radius[j] = 0;
		    speed_mat[j] = atan(relative_location_y/(set_radius-relative_location_x))/time_constant/2*wheel_distance;
		    j++;
		} else if (LOC_ORI[i] == 1){
		    command[j] = atan(relative_location_y/(set_radius+relative_location_x))/3.14159265358979328626*180;
		    LOC[j] = LOC_ORI[i];
		    radius[j] = 0;
		    speed_mat[j] = atan(relative_location_y/(set_radius+relative_location_x))/time_constant/2*wheel_distance;
		    j++;

		    command[j] = center_command[i];
		    LOC[j] = LOC_ORI[i];
		    radius[j] = set_radius;
		    speed_mat[j] = set_turn_speed;
		    j++;

		    command[j] = atan(relative_location_y/(set_radius+relative_location_x))/3.14159265358979328626*180;
		    LOC[j] = 3-LOC_ORI[i];
		    radius[j] = 0;
		    speed_mat[j] = atan(relative_location_y/(set_radius+relative_location_x))/time_constant/2*wheel_distance;
		    j++;
		}
	    }
        } else {
          timer -= 1;
          if (timer % 10 == 0) {
            printf("timer is: %d\n", timer);
          }
          state = GETTING_NUM;
          kobukiDriveDirect(0, 0, -1);
        }
        break;
      }

    case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
            state = START;
            counter = 0;
            initial_encoder = sensors.rightWheelEncoder;
            measure_distance_or_angle = 0;
        } else {
            display_write("OFF", DISPLAY_LINE_0);
            printf("\n");
            state = OFF;
            // perform state-specific actions here
            kobukiDriveDirect(0, 0, -1);
        }
        break; // each case needs to end with break!
    }

    case START: {
        // transition logic
    if (counter >= m){
        state = OFF;
        counter = 0;
    } else if (LOC[counter] == 1){
        state = LEADER_TURNLEFT;
        next_state = START;
        measure_distance_or_angle = 0;
        set_distance_or_angle = command[counter];
        rad = radius[counter];
        spd = speed_mat[counter];
        initial_encoder = sensors.rightWheelEncoder;
        lsm9ds1_start_gyro_integration();
        counter = counter + 1;
    } else if (LOC[counter] == 0){
        state = LEADER_FORWARD;
        next_state = START;
        measure_distance_or_angle = 0;
        set_distance_or_angle = command[counter];
        rad = radius[counter];
        spd = speed_mat[counter];
        initial_encoder = sensors.rightWheelEncoder;
        counter = counter + 1;
    } else if (LOC[counter] == 2){
        state = LEADER_TURNRIGHT;
        next_state = START;
        measure_distance_or_angle = 0;
        set_distance_or_angle = command[counter];
        rad = radius[counter];
        spd = speed_mat[counter];
        initial_encoder = sensors.rightWheelEncoder;
        lsm9ds1_start_gyro_integration();
        counter = counter + 1;
    }
    break; // each case needs to end with break!
    }


    case LEADER_FORWARD: {
        //variable: set_speed, set_distance_or_angle, next_state, rad, spd
        // transition logic
        if (is_button_pressed(&sensors)) {
            state = OFF;
        } else if (measure_distance_or_angle >= set_distance_or_angle-0.5*spd/1000*1/POSE_UPDATE_HZ){
            state = next_state;
            kobukiDriveDirect(0, 0, -1);
            measure_distance_or_angle = 0;
            initial_encoder = sensors.rightWheelEncoder;
        lsm9ds1_stop_gyro_integration();
	} else {
        display_write("LEADER_FORWARD", DISPLAY_LINE_0);
        kobukiDriveDirect(spd, spd, rad);
        measure_distance_or_angle = get_distance(sensors.rightWheelEncoder, initial_encoder);
        char line[16];
        snprintf(line, 16, "%f", measure_distance_or_angle);
        display_write(line, DISPLAY_LINE_1);
        printf("\n");
        }
        break; // each case needs to end with break!
    }


    case LEADER_TURNLEFT: {
        //variable: set_speed, set_distance_or_angle, set_radius, next_state, rad, spd
        // transition logic

        if (is_button_pressed(&sensors)) {
            state = OFF;
        lsm9ds1_stop_gyro_integration();
        } else if (measure_distance_or_angle >= (set_distance_or_angle-180/3.1415926535898*0.5*spd/1000/set_radius*1/POSE_UPDATE_HZ)){
            state = next_state;
            kobukiDriveDirect(0, 0, -1);
            measure_distance_or_angle=0;
            initial_encoder = sensors.rightWheelEncoder;
            lsm9ds1_stop_gyro_integration();
        } else {
            display_write("LEADER_TURNLEFT", DISPLAY_LINE_0);
            if (rad!=0){
                v1 = spd/rad*(sqrt(pow(relative_location_y,2) + pow(rad+relative_location_x,2))-0.5*wheel_distance);
                v2 = spd/rad*(sqrt(pow(relative_location_y,2) + pow(rad+relative_location_x,2))+0.5*wheel_distance);
                kobukiDriveDirect(v1, v2, sqrt(pow(set_radius+relative_location_x,2)+pow(relative_location_y,2)));
                lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
                measure_distance_or_angle = meas.z_axis;
                char line[16];
                snprintf(line, 16, "%f", measure_distance_or_angle);
                display_write(line, DISPLAY_LINE_1);
                printf("\n");
            } else {
                v1 = -spd;
                v2 = spd;
                kobukiDriveDirect(v1, v2, rad);
                lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
                measure_distance_or_angle = meas.z_axis;
                char line[16];
                snprintf(line, 16, "%f", measure_distance_or_angle);
                display_write(line, DISPLAY_LINE_1);
                printf("\n");
            }
        }

    break; // each case needs to end with break!
    }


    case LEADER_TURNRIGHT: {
        //variable: set_speed, set_distance_or_angle, set_radius, next_state, rad, spd
        // transition logic
        if (is_button_pressed(&sensors)) {
            state = OFF;
        lsm9ds1_stop_gyro_integration();
        } else if (measure_distance_or_angle <= -(set_distance_or_angle-180/3.1415926535898*0.5*spd/1000/set_radius*1/POSE_UPDATE_HZ)){
            state = next_state;
            kobukiDriveDirect(0, 0, -1);
            measure_distance_or_angle=0;
            initial_encoder = sensors.rightWheelEncoder;
            lsm9ds1_stop_gyro_integration();
        } else {
            display_write("LEADER_TURNRIGHT", DISPLAY_LINE_0);
            if (rad!=0){
                v1 = spd/rad*(sqrt(pow(relative_location_y,2) + pow(rad-relative_location_x,2))+0.5*wheel_distance);
                v2 = spd/rad*(sqrt(pow(relative_location_y,2) + pow(rad-relative_location_x,2))-0.5*wheel_distance);
                kobukiDriveDirect(v1, v2, sqrt(pow(rad+relative_location_x,2)+pow(relative_location_y,2)));
                lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
                measure_distance_or_angle = meas.z_axis;
                char line[16];
                snprintf(line, 16, "%f", measure_distance_or_angle);
                display_write(line, DISPLAY_LINE_1);
                printf("\n");
            } else {
                v1 = spd;
                v2 = -spd;
                kobukiDriveDirect(v1, v2, rad);
                lsm9ds1_measurement_t meas = lsm9ds1_read_gyro_integration();
                measure_distance_or_angle = meas.z_axis;
                char line[16];
                snprintf(line, 16, "%f", measure_distance_or_angle);
                display_write(line, DISPLAY_LINE_1);
                printf("\n");
            }
        }
        break; // each case needs to end with break!
    }

    }
    return state;
}

