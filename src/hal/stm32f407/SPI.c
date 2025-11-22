#include "../../communication/SPI.h"
#include "../../communication/usart.h"
#include "../../globals.h"

#include "../../hw_init.h"
#include "./Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
#include "../../sensors/LSM9DS1.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

//volatile SPI_State_t spi_current_state = SPI_STATE_IDLE;
volatile int spi_next_state = SPI_STATE_IDLE;
volatile uint8_t spi_busy = 0;
volatile uint8_t lsm9_has_init = 0;
volatile uint8_t lsm9_who_am_i_correct = 0;

SPI_INFO SPI_current;

void CS1_Select() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}
void CS1_Unselect() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

void CS2_Select() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}
void CS2_Unselect() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

void CS3_Select() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
}
void CS3_Unselect() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
}

void CS_A_H(void) {
    CS1_Unselect();
}

void CS_A_L(void) {
    CS1_Select();
}

void CS_M_H(void) {
    CS2_Unselect();
}

void CS_M_L(void) {
    CS2_Select();
}


void dma2_stream3_handler() {
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void dma2_stream0_handler() {
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}


void SPI_select() {
    if (spi_busy)
        return;
    // lsm9 init statemachine
    if (!lsm9_has_init) {
        if (spi_next_state >= SPI_STATE_LSM9_INIT_DONE) {
            lsm9_has_init = 1;
            spi_next_state++;
            return;
        } else {
            spi_next_state++;
            return;
        }
    }

    if (spi_next_state == SPI_STATE_LSM9_INIT_DONE) {
        spi_next_state++;
        return;
    }

    // lsm9 who am i state machine
    if (!lsm9_who_am_i_correct) {
        // called only when when who am I is wrong
        if (spi_next_state == SPI_STATE_LSM9_READ_WHO_AM_I_M) {
            os_printf("[SPI] ERROR: WHO_AM_I wrong! \n");
            return;
        } else {
            // should only be called once since previous state machine calles the first who am i
            // funciton
            spi_next_state++;
            return;
        }
    }
    if (lsm9_who_am_i_correct && spi_next_state == SPI_STATE_LSM9_READ_WHO_AM_I_M) {
        spi_next_state++;
    }

    // state machine cases
    switch (spi_next_state) {
    case SPI_STATE_LSM9_READ_GYRO:
        spi_next_state++;
        break;
    case SPI_STATE_LSM9_READ_ACC:
        spi_next_state++;
        break;
    case SPI_STATE_LSM9_READ_MAG:
        spi_next_state = SPI_STATE_LSM9_READ_GYRO;
        break;
    }
    if( spi_next_state > SPI_STATE_LSM9_READ_MAG) {
        spi_next_state = SPI_STATE_LSM9_READ_GYRO;
    }

    return;
}

void SPI_state_machine() {
    switch (spi_next_state) {
    case SPI_STATE_IDLE:
        return;
    case SPI_STATE_LSM9_RESET:
        LSM9DS1_READ_CTRL_REG4_M(&SPI_current);
        //LSM9DS1_reset(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG1_G:
        os_printf("ctrl write \n");;
        LSM9DS1_WRITE_CTRL_REG1_G(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG3_G:
        LSM9DS1_WRITE_CTRL_REG3_G(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG6_XL:
        LSM9DS1_WRITE_CTRL_REG6_XL(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG1_M:
        LSM9DS1_WRITE_CTRL_REG1_M(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG2_M:
        LSM9DS1_WRITE_CTRL_REG2_M(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG3_M:
        LSM9DS1_WRITE_CTRL_REG3_M(&SPI_current);
        break;
    case SPI_STATE_LSM9_WRITE_CTRL_REG4_M:
        LSM9DS1_WRITE_CTRL_REG4_M(&SPI_current);
        break;
    case SPI_STATE_LSM9_INIT_DONE:
        break;
    case SPI_STATE_LSM9_READ_WHO_AM_I_A:
        LSM9DS1_read_WHO_AM_I_A(&SPI_current);
        break;
    case SPI_STATE_LSM9_READ_WHO_AM_I_M:
        LSM9DS1_read_WHO_AM_I_M(&SPI_current);
        break;
    case SPI_STATE_LSM9_READ_GYRO:
        LSM9DS1_read_gyro(&SPI_current);
        break;
    case SPI_STATE_LSM9_READ_ACC:
        LSM9DS1_read_acc(&SPI_current);
        break;
    case SPI_STATE_LSM9_READ_MAG:
        LSM9DS1_read_mag(&SPI_current);
        break;
    }

    //LSM9DS1_READ_CTRL_REG4_M(&SPI_current);
    //LSM9DS1_WRITE_CTRL_REG1_G(&SPI_current);

    return;
}

int trans_cnt = 0;
int whoami = 0;
int who_am_i_a_correct = 0;
int who_am_i_m_correct = 0;
void SPI_send() {
    if (spi_busy || spi_next_state == SPI_STATE_IDLE) return;
    spi_busy = 1;

    // sending logic
    SPI_current.cs_low();

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_current.tx, SPI_current.rx, SPI_current.size) != HAL_OK) {
        SPI_current.cs_high();
        os_printf("[SPI] Transmission error! CNT: %d\n", trans_cnt);
        spi_busy = 0;
        return;
    }

    trans_cnt++;
    return;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {

        whoami = SPI_current.rx[1]; // response byte
        if (spi_next_state == SPI_STATE_LSM9_READ_WHO_AM_I_A) {
            if (whoami == 104) {
                who_am_i_a_correct = 1;
            }
        }
        if (spi_next_state == SPI_STATE_LSM9_READ_WHO_AM_I_M) {
            if (whoami == 61) {
                who_am_i_m_correct = 1;
                lsm9_who_am_i_correct = 1;
            }
        }
        SPI_current.cs_high();
        CS_A_H();
        CS_M_H();
        os_printf("res: ");
        for (int i = 0; i < SPI_current.size; ++i) {
            os_printf("%d ", SPI_current.rx[i]);
        }
        os_printf("\n");
        spi_busy = 0;
        //SPI_process();
    }
}


void SPI_process() {
    if (spi_busy) return;
    SPI_select();
    SPI_state_machine();
    SPI_send();
}

void SPI_thread() {
    //spi_current_state = SPI_STATE_IDLE;
    spi_next_state = SPI_STATE_IDLE;
        CS_A_H();
        CS_M_H();

    while (1) {
        if (!spi_busy) {
            SPI_process();
        }

        sleep(1 * MILLISECONDS);
    }
}
