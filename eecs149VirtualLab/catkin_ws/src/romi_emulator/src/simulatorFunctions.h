#include "kobukiSensorTypes.h"

#ifdef __cplusplus
extern "C"
{
#endif
void kobukiPositionPoll(robot_position_t* positions);
void display_write(const char *format, display_line line);
lsm9ds1_measurement_t lsm9ds1_read_accelerometer();
void kobukiSensorPoll(KobukiSensors_t * const sensors);
void nrf_delay_ms(uint32_t volatile delay);
bool is_button_pressed(KobukiSensors_t* sensors);
uint32_t lsm9ds1_start_gyro_integration();
void lsm9ds1_stop_gyro_integration();
int32_t kobukiDriveDirect(int16_t leftWheelSpeed, int16_t rightWheelSpeed);
lsm9ds1_measurement_t lsm9ds1_read_gyro_integration();

#ifdef __cplusplus
}
#endif
