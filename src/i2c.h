#ifndef __i2c_imu
#define __i2c_imu
#include <stdint.h>

// PB11 is SDA
// PB10 is SCL
// this uses i2c2

void configure_i2c();
void start_i2c_communication();
//returns 0 if ack, 1 if nack
uint8_t i2c_send_data(uint8_t adress, uint8_t data);
uint8_t i2c_send_data_multiple(uint8_t adress, uint8_t * data, uint32_t size);
uint8_t * i2c_recieve_data(uint8_t adress, uint32_t size);

#endif
