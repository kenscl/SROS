#ifndef SPI
#define SPI
#include <stddef.h>
#include <stdint.h>
#include "../krnl/scheduler.h"

#include "./Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern volatile uint8_t spi_busy;

extern uint32_t SPI_error_count;

typedef enum {
SPI_STATE_IDLE = 0,
  //// LSM9 initi stuff
  SPI_STATE_LSM9_RESET,
  SPI_STATE_LSM9_WRITE_CTRL_REG1_G,
  SPI_STATE_LSM9_WRITE_CTRL_REG3_G,
  SPI_STATE_LSM9_WRITE_CTRL_REG6_XL,
  SPI_STATE_LSM9_WRITE_CTRL_REG1_M,
  SPI_STATE_LSM9_WRITE_CTRL_REG2_M,
  SPI_STATE_LSM9_WRITE_CTRL_REG3_M,
  SPI_STATE_LSM9_WRITE_CTRL_REG4_M,

  //// LSM9 initi last state
SPI_STATE_LSM9_INIT_DONE,

  //// LSM9 comms whoami stuff
SPI_STATE_LSM9_READ_WHO_AM_I_A,
SPI_STATE_LSM9_READ_WHO_AM_I_M,

  //// all cases here will be called periodically. They should therefor always be at the end
  //// LSM9 comms stuff
SPI_STATE_LSM9_READ_GYRO,
SPI_STATE_LSM9_READ_ACC,
SPI_STATE_LSM9_READ_MAG,
} SPI_State_t;

extern volatile SPI_State_t spi_current_state;

typedef struct {
uint8_t *rx;
uint8_t *tx;

void (*cs_low)(void);
void (*cs_high)(void);

size_t size;
} SPI_INFO;

extern SPI_INFO SPI_current;

void SPI_state_machine();
void SPI_select();
void SPI_send();
void SPI_process();
void SPI_reconnect();
volatile void SPI_thread();

#endif
