#include "i2c.h"
#include <stm32f1xx.h>
#include <stdint.h>
#include "mem.h"
#include "usart.h"

void configure_i2c() {
    I2C2->CR1 &= ~(1 << 0); // disable i2c for config 

    // reset i2c2
    I2C2->CR1 |= (1 << 15);
    I2C2->CR1 &= ~(1 << 15);
    
    // gpio config and so on
    RCC->APB2ENR |= (1 << 3); // enable IOPBEN
    
    // clock config for I2C2
    // I2C2 is on the APB1 clock.
    RCC->APB1ENR |= (1 << 22); // enable clock for i2c2;
    RCC->APB2ENR |= (1 << 3); // enable clock for GPIOB;
    RCC->APB2ENR |= (1 << 0); // enable clock for AFIO;

    GPIOB->CRH &= ~(0xF << 8);  // Clear MODE10
    GPIOB->CRH |= (0xB << 8);   // Set MODE10 to Alternate function open drain
    GPIOB->CRH &= ~(0xF << 12); // Clear MODE11
    GPIOB->CRH |= (0xB << 12);  // Set MODE11 to Alternate function open drain

    // speed is 36 MHz so 0b100100
    uint32_t speed = 0b100100;
    I2C2->CR2 = (I2C2->CR2 & ~I2C_CR2_FREQ_Msk) | (speed & I2C_CR2_FREQ_Msk); // write bit field

    // configure the i2c2 ccr
    I2C2->CCR &= ~(1 << 15); // set standard mode
    // target is 100kHz SCL, so 10 µs
    // pclk is 36 MHz, so T_pclk is 27,78 ns.
    // T_SCL = T_pclk * CCR
    // -> CCR = T_SCL / T_pclk = 10 µs / 27,78 ns = 359.97
    speed = 360;
    I2C2->CCR = (I2C2->CCR & ~I2C_CCR_CCR_Msk) | (speed & I2C_CCR_CCR_Msk);

    // configure the i2c Trise
    // time should be 1000 ns / T_pclk + 1 = 37; 
    speed = 37;
    I2C2->TRISE = (I2C2->TRISE & ~I2C_TRISE_TRISE_Msk) | (speed & I2C_TRISE_TRISE_Msk);

    // program the I2C_CR1 register to enable the peripheral
    I2C2->CR1 &= ~(1 << 1); // i2c mode
    I2C2->CR1 |= (1 << 0); // i2c enable 
}

void start_i2c_communication() {
    I2C2->CR1 |= (1 << 8); // generate start condition
    while (!(I2C2->SR1 & (1<<0))); // wait for start condition generated
}

void i2c_send_adress(uint8_t adress) {
    I2C2->DR = adress;
    while (!(I2C2->SR1 & (1<<1))); // wait for adress bit to set 
    uint8_t temp = I2C2->SR1 | I2C2->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void i2c_stop() {
    I2C2->CR1 |= (1<<9); // stop bit
}

void i2c_send_data(uint8_t data) {
    while (!(I2C2->SR1 & (1<<7))); // wait for tx to finish 
    I2C2->DR = data;
    while (!(I2C2->SR1 & (1<<2))); // wait for transfer to finish
}

void i2c_send_data_multiple(uint8_t * data, uint32_t size) {
    for (int i = 0; i < size; ++i) {
        while (!(I2C2->SR1 & (1<<7))); // wait for tx to finish 
        I2C2->DR = data[i];
    }
    while (!(I2C2->SR1 & (1<<2))); // wait for transfer to finish

    I2C2->CR1 |= (1<<9); // stop bit
}

uint8_t * i2c_recieve_data(uint8_t adress, uint32_t size) {
    uint8_t * buffer = (uint8_t * ) os_alloc (size);
    for(int i = 0; i < size; ++i) {
        buffer[i] = 0;
    }
    if (buffer == 0) return nullptr;

    // case 1 byte
    if (size == 1) {
        // send slave adress
        I2C2->DR = adress;
        //while (!(I2C2->SR1 & (1<<2))); // wait for transfer to finish
        //I2C2->CR1 &= ~(1 << 10); // clear ack bit
        uint8_t dummy = I2C2->SR1 | I2C2->SR2; // clear addr bit
        I2C2->CR1 &= ~(1<<8); // clear start bit
        I2C2->CR1 |= (1<<9); // stop bit
        while (!(I2C2->SR1 & (1<<6))); // wait for RxNE to set
        buffer[0] = I2C2->DR;
    }
    // multiple bytes
    else {
        // send slave adress
        I2C2->DR = adress;
        while (!(I2C2->SR1 & (1<<1))); // wait for transfer to finish
        uint8_t dummy = I2C2->SR1 | I2C2->SR2; // clear addr bit
        for (int i = 0; i < size; ++i) {
            while (!(I2C2->SR1 & (1<<6))); // wait for RxNE to set
            buffer[i] = I2C2->DR; // read data
            // acknolage 
            if (i > 2) {
                I2C2->CR1 |= (1 << 10); // clear ack bit
            }
            else if (i == 2) { // for the second last transfer clear the bit
                I2C2->CR1 &= ~(1 << 10); // clear ack bit
                I2C2->CR1 |= (1<<9); // stop bit
            }
        }
    }
    return buffer;
}
