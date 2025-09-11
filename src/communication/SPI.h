#ifndef SPI
#define SPI
#include <cstddef>
#include <cstdint>
#include <stdint.h>
#include "../utils/queue.h"

extern "C" {
void dma2_stream0_handler();
}

enum class SPI_state { pending, busy, done, error };

typedef struct SPI_transmition {
  uint8_t *rx_buffer;
  uint8_t *tx_buffer;
  size_t size;

  void (*cs_low)(void);
  void (*cs_high)(void);

  volatile SPI_state state = SPI_state::pending;
}SPI_transmition;

class SPI_manager {
  protected:
  Queue<volatile SPI_transmition> queue;

  public:
  void init();
  int submit(SPI_transmition * tx);
  void handle();

  friend void dma2_stream0_handler();
} extern OS_SPI_manager;

#endif 
