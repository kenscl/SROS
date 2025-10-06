#ifndef SPI
#define SPI
#include <stddef.h>
#include <stdint.h>

void dma2_stream0_handler();

enum SPI_state { pending, busy, done, error };

typedef struct SPI_transmition {
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    size_t size;

    void (*cs_low)(void);
    void (*cs_high)(void);

    volatile enum SPI_state state; // = SPI_state::pending; set as default
} SPI_transmition;

void SPI_init();
int SPI_submit(volatile struct SPI_transmition *tx);
void SPI_handle();

#endif
