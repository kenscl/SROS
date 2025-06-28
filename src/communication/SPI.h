#ifndef SPI
#define SPI
#include <cstddef>
#include <cstdint>
#include <stdint.h> 

enum SPI_state{
  Read,
  Write,
  Done
};

enum SPI_target {
  Accelerometer,
  Magnetometer
};

struct SPI_information {
  SPI_state state;
  size_t size;
  uint8_t * rx_buffer;
  uint8_t * tx_buffer;
  SPI_target target;
  uint8_t adress;
};

// use spi for communication
uint8_t SPI_handle(SPI_information * info);
void SPI_send_next();
void SPI_init();
void CS_A_L();
void CS_A_H();
void CS_M_L();
void CS_M_H();
volatile void SPI_thread();

#endif 
