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

struct SPI_target {
  void (*cs_low)(void);
  void (*cs_high)(void);
  void (*send_data)(uint8_t adress, uint8_t * rx_buffer, uint8_t * tx_buffer, size_t size);
};

struct SPI_information {
  SPI_state state;
  size_t size;
  uint8_t * rx_buffer;
  uint8_t * tx_buffer;
  SPI_target * target;
  uint8_t adress;
};

uint8_t SPI_handle(SPI_information * info);
void SPI_send_next();
void SPI_init();
volatile void SPI_thread();

#endif 
