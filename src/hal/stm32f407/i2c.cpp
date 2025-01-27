#include "../../communication/i2c.h"
#include <stm32f4xx.h>
#include <stdint.h>
#include "../../krnl/mem.h"
#include "../../communication/usart.h"

void configure_i2c() {
}

void start_i2c_communication() {
}

void i2c_send_adress(uint8_t adress) {
}

void i2c_stop() {
}

void i2c_send_data(uint8_t data) {
}

void i2c_send_data_multiple(uint8_t * data, uint32_t size) {
}

uint8_t * i2c_recieve_data(uint8_t adress, uint32_t size) {
 uint8_t * buffer = (uint8_t *)os_alloc(size);
 return buffer;
}
