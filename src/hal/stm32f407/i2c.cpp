#include "../../communication/i2c.h"
#include <cstdint>
#include <stm32f4xx.h>
#include <stdint.h>
#include "../../krnl/mem.h"
#include "../../krnl/scheduler.h"
#include "../../communication/usart.h"

void configure_i2c() {
    I2C1->CR1 &= ~(1 << 0); // disable i2c for config 

    // reset i2c1
    I2C1->CR1 |= (1 << 15);
    I2C1->CR1 &= ~(1 << 15);
    
    // gpio config and so on
    RCC->APB2ENR |= (1 << 3); // enable IOPBEN
    
    // clock config for I2C1
    // I2C1 is on the APB1 clock.
    RCC->APB1ENR |= (1 << 21); // enable clock for i2c1;
    RCC->AHB1ENR |= (1 << 1); // enable clock for GPIOB;

    GPIOB->AFR[0] |= (4 << (4 * 6)); // Set AF4 
    GPIOB->AFR[0] |= (4 << (4 * 7)); // Set AF4 
    GPIOB->OTYPER |= (1 << 6) | (1 << 7); // open-drain for I2C
    GPIOB->PUPDR &= ~((0b11 << 12) | (0b11 << 14)); // clea for PB6, PB7
    GPIOB->PUPDR |= (0b01 << 12) | (0b01 << 14);    // Set pull-up mode

    GPIOB->MODER &= ~(0b11 << 12);  // Clear MODE6
    GPIOB->MODER |= (0b10 << 12);   // Set MODE6 to Alternate function open drain
    GPIOB->MODER &= ~(0b11 << 14); // Clear MODE7
    GPIOB->MODER |= (0b10 << 14);  // Set MODE7 to Alternate function open drain

    I2C1->CR1 |= (1 << 15); // reset i2c befor config 
    I2C1->CR1 &= ~(1 << 15); // clear reset i2c 

    // speed is 42 MHz so 0b101010
    uint32_t speed = 0b101010;
    I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ_Msk) | (speed & I2C_CR2_FREQ_Msk); // write bit field

    // configure the i2c1 ccr
    I2C1->CCR &= ~(1 << 15); // set standard mode
    // target is 100kHz SCL, so 10 µs
    // pclk is 42 MHz, so T_pclk is 23,81 ns.
    // T_SCL = T_pclk * CCR
    // -> CCR = T_SCL / T_pclk = 10 µs / 23,81 ns = 420
    speed = 420;
    I2C1->CCR = (I2C1->CCR & ~I2C_CCR_CCR_Msk) | (speed & I2C_CCR_CCR_Msk);

    // configure the i2c Trise
    // time should be 1000 ns / T_pclk + 1 = 40; 
    speed = 40;
    //I2C1->TRISE = (I2C1->TRISE & ~I2C_TRISE_TRISE_Msk) | (speed & I2C_TRISE_TRISE_Msk);

    // program the I2C_CR1 register to enable the peripheral
    I2C1->CR1 &= ~(1 << 1); // i2c mode
    I2C1->CR1 |= (1 << 0); // i2c enable 
}

void start_i2c_communication() {
    scheduler_disable();
    os_interrupt_disable();
    //if (I2C1->SR2 & (1 << 1)) { // Bus is busy, need to reset the i2c device
    //    i2c_recover(); // this causes more issues that it solves
    //    configure_i2c();
    //}
    I2C1->CR1 |= (1 << 8); // generate start condition
    while (!(I2C1->SR1 & (1<<0))) {
    } // wait for start condition generated
    os_interrupt_enable();
    scheduler_enable();
}

void i2c_send_adress(uint8_t adress) {
    scheduler_disable();
    os_interrupt_disable();
    I2C1->DR = adress;
    while (!(I2C1->SR1 & (1<<1))); // wait for adress bit to set 
    uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
    os_interrupt_enable();
    scheduler_enable();
}

void i2c_stop() {
    scheduler_disable();
    os_interrupt_disable();
    I2C1->CR1 |= (1<<9); // stop bit
    while(!( I2C1->CR1 & (1<<9))) ;
    os_interrupt_enable();
    scheduler_enable();
}

void i2c_send_data(uint8_t data) {
    scheduler_disable();
    os_interrupt_disable();
    while (!(I2C1->SR1 & (1<<7))); // wait for tx to finish 
    I2C1->DR = data;
    while (!(I2C1->SR1 & (1<<2))); // wait for transfer to finish
    os_interrupt_enable();
    scheduler_enable();
}

void i2c_send_data_multiple(uint8_t * data, uint32_t size) {
    scheduler_disable();
    os_interrupt_disable();
    for (int i = 0; i < size; ++i) {
        while (!(I2C1->SR1 & (1<<7))); // wait for tx to finish 
        I2C1->DR = data[i];
    }
    while (!(I2C1->SR1 & (1<<2))); // wait for transfer to finish

    I2C1->CR1 |= (1<<9); // stop bit
    os_interrupt_enable();
    scheduler_enable();
}

uint8_t * i2c_recieve_data(uint8_t adress, uint32_t size) {
    scheduler_disable();
    os_interrupt_disable();
    uint8_t * buffer = (uint8_t *)os_alloc(size);
    if (!buffer) {
        OS_WARN("Cannot allocate ");
        os_interrupt_enable();
    scheduler_enable();
        return NULL;
    }

    I2C1->DR = adress; // Send address
    while (!(I2C1->SR1 & (1 << 1))); // Wait for ADDR
    uint8_t dummy = I2C1->SR1 | I2C1->SR2; // Clear ADDR

    if (size == 1) {
        I2C1->CR1 &= ~(1 << 10); // Disable ACK
        I2C1->CR1 |= (1 << 9); // Generate STOP
        while (!(I2C1->SR1 & (1 << 6))); // Wait for RxNE
        buffer[0] = I2C1->DR; // Read data
    } else {
        I2C1->CR1 |= (1 << 10); // Enable ACK
        for (uint32_t i = 0; i < size; i++) {
            while (!(I2C1->SR1 & (1 << 6))); // Wait for RxNE
            buffer[i] = I2C1->DR; // Read data

            if (i == size - 2) I2C1->CR1 &= ~(1 << 10); // Disable ACK before last byte
            else if (i == size - 1) I2C1->CR1 |= (1 << 9); // Generate STOP
        }
    }

    os_interrupt_enable();
    scheduler_enable();
    return buffer;
}
