#include "../../communication/i2c.h"
#include "../../communication/usart.h"
#include "../../krnl/mem.h"
#include "../../krnl/scheduler.h"
#include <cstdint>
#include <stdint.h>
#include <stm32f4xx.h>
#define I2C_QUEUE_LENGTH 20
I2C_state_information dummy;
I2C_state_information *current = &dummy;

I2C1_state_enum I2C1_state;

extern "C" {
#pragma GCC push_options
#pragma GCC optimize("O0") // otherwise the compiler will mangle the function
volatile uint8_t i2c1_reset = 0;
volatile void i2c1_ev_handler(void) {
  volatile static uint32_t i = 0;
  volatile static uint32_t cnt = 0;
  volatile uint16_t status = I2C1->SR1;
  volatile uint8_t temp = 0;
  volatile uint8_t start = ((I2C1->SR1 & (1 << 0)) != 0);
  volatile uint8_t addr = ((I2C1->SR1 & (1 << 1)) != 0);
  volatile uint8_t btf = ((I2C1->SR1 & (1 << 2)) != 0);
  volatile uint8_t rxne = ((I2C1->SR1 & (1 << 6)) != 0);
  volatile uint8_t tx = ((I2C1->SR1 & (1 << 7)) != 0);
  if (i2c1_reset) {
      cnt = 0;
      i2c1_reset = 0;
      i = 0;
  }
  if (current->state == Done) {
    configure_i2c();
    return;
  }
  // if (btf) return;
  //  transmition
  if (current->state == Sending) {
    // st handled outsied
    // SAD + W
    if (start && (cnt == 0)) {
      i2c_send_data(current->device_adress_write);
      cnt++;
      return;
    }
    // SUB
    if (tx && (cnt == 1)) {
      temp = I2C1->SR1 | I2C1->SR2;
      i2c_send_data(current->device_subadress);
      cnt++;
      return;
    }
    // DATA
    if (tx && (cnt == 2)) {
      i2c_send_data(current->data[0]);
      cnt++;
      return;
    }
    // SP
    if (cnt == 3) {
      current->state = Done;
      i2c_stop();
      cnt = 0;
      return;
    }
  }
  // reception single
  if (current->state == Recieving) {
    // st handled outside
    // SAD + W
    if (start && (cnt == 0)) {
      i2c_send_data(current->device_adress_write);
      cnt++;
      return;
    }
    // SUB
    if (tx && (cnt == 1)) {
      temp = I2C1->SR1 | I2C1->SR2;
      i2c_send_data(current->device_subadress);
      cnt++;
      return;
    }
    // SR
    if (tx && (cnt == 2)) {
      start_i2c_communication();
      cnt++;
      return;
    }
    // SAD + R
    if (tx && (cnt == 3)) {
      for (volatile int i = 0; i < 500; i++)
        ; // delay needed for some reason :( will keep it for now, but will have
          // to invastigate in the future
      I2C1->DR = current->device_adress_recieve;
      cnt++;
      return;
    }
    // rec
    if (addr && (cnt == 4)) {
      temp = I2C1->SR1 | I2C1->SR2;
      cnt++;
      if (current->recieve_bytes == 1) {
        i2c_recieve_single();
        return;
      } else {
        I2C1->CR1 |= (1 << 10); // Enable ACK
        return;
      }
    }
    // rec 2
    if (rxne) {
      if (current->recieve_bytes == 1) {
        current->data[0] = (uint8_t)I2C1->DR;
        current->state = Done;
        i2c_stop();
        cnt = 0;
        return;
      } else {
        while (i < current->recieve_bytes) {
          current->data[i] = (uint8_t)I2C1->DR;
          if (i == current->recieve_bytes - 2)
            I2C1->CR1 &= ~(1 << 10); // Disable ACK before last byte
          else if (i == current->recieve_bytes - 1) {
            current->state = Done;
            i2c_stop();
            cnt = 0;
            i = 0;
            return;
          }
          ++i;
          return;
        }
      }
    }
    return;
  }
}

volatile void i2c1_er_handler(void) {
  volatile uint32_t reg = I2C1->SR1;
  i2c_stop();

  if ((reg & (1 << 8)) != 0) { // BERR
    reg &= ~(1 << 8);
    os_printf("BERR \n");
    i2c1_reset = 1;
    start_i2c_communication(); // resetart with current I2C_state_information in
                               // case of issues
  }
  if ((reg & (1 << 9)) != 0) { // ARLO
    reg &= ~(1 << 9);
    os_printf("ARLO \n");
    i2c1_reset = 1;
    start_i2c_communication(); // resetart with current I2C_state_information in
  }
  if ((reg & (1 << 10)) != 0) { // AF
    reg &= ~(1 << 10);
    os_printf("AF \n");
  }
}

#pragma GCC pop_options
}

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
  RCC->AHB1ENR |= (1 << 1);  // enable clock for GPIOB;

  GPIOB->AFR[0] |= (4 << (4 * 6));                // Set AF4
  GPIOB->AFR[0] |= (4 << (4 * 7));                // Set AF4
  GPIOB->OTYPER |= (1 << 6) | (1 << 7);           // open-drain for I2C
  GPIOB->PUPDR &= ~((0b11 << 12) | (0b11 << 14)); // clea for PB6, PB7
  GPIOB->PUPDR |= (0b01 << 12) | (0b01 << 14);    // Set pull-up mode

  GPIOB->MODER &= ~(0b11 << 12); // Clear MODE6
  GPIOB->MODER |= (0b10 << 12);  // Set MODE6 to Alternate function open drain
  GPIOB->MODER &= ~(0b11 << 14); // Clear MODE7
  GPIOB->MODER |= (0b10 << 14);  // Set MODE7 to Alternate function open drain

  I2C1->CR1 |= (1 << 15);  // reset i2c befor config
  I2C1->CR1 &= ~(1 << 15); // clear reset i2c

  // speed is 42 MHz so 0b101010
  uint32_t speed = 0b101010;
  I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ_Msk) |
              (speed & I2C_CR2_FREQ_Msk); // write bit field

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
  // I2C1->TRISE = (I2C1->TRISE & ~I2C_TRISE_TRISE_Msk) | (speed &
  // I2C_TRISE_TRISE_Msk);

  //  interrupts
  I2C1->CR2 |= 1 << 9; // Event interrupt enable
  I2C1->CR2 |= 1 << 8; // Error interrupt enable

  // program the I2C_CR1 register to enable the peripheral
  I2C1->CR1 &= ~(1 << 1); // i2c mode
  I2C1->CR1 |= (1 << 0);  // i2c enable
  I2C1_state = Idle;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
  NVIC_SetPriority(I2C1_EV_IRQn, 3);
  NVIC_SetPriority(I2C1_ER_IRQn, 4);
}

void start_i2c_communication() {
  I2C1->CR1 |= (1 << 8); // generate start condition
  return;
}

void i2c_send_adress(uint8_t adress) {
  I2C1->DR = adress;
  return;
}

void i2c_stop() {
  I2C1->CR1 |= (1 << 9); // stop bit
  while (!(I2C1->CR1 & (1 << 9)))
    ;
}

void i2c_send_data(uint8_t data) {
  I2C1->DR = data;
  return;
}

void i2c_send_data_multiple(uint8_t *data, uint32_t size) {
  for (int i = 0; i < size; ++i) {
    while (!(I2C1->SR1 & (1 << 7)))
      ; // wait for tx to finish
    I2C1->DR = data[i];
  }
  while (!(I2C1->SR1 & (1 << 2)))
    ; // wait for transfer to finish

  I2C1->CR1 |= (1 << 9); // stop bit
  return;
}

void i2c_recieve_single() {
  I2C1->CR1 &= ~(1 << 10); // Disable ACK
  I2C1->CR1 |= (1 << 9);   // Generate STOP
  return;
}

struct I2C_queue {
  I2C_state_information *info = nullptr;
  uint8_t occupied = 0;
};

I2C_queue queue[I2C_QUEUE_LENGTH];
uint16_t queue_head = 0;

uint8_t enqueue(I2C_state_information *info) {
  for (int i = queue_head; i < I2C_QUEUE_LENGTH + queue_head; ++i) {
    if (i < I2C_QUEUE_LENGTH) {
      if (!queue[i].occupied) {
        queue[i].info = info;
        queue[i].occupied = 1;
        return 1;
      }
    } else {
      if (!queue[i - I2C_QUEUE_LENGTH].occupied) {
        queue[i - I2C_QUEUE_LENGTH].info = info;
        queue[i - I2C_QUEUE_LENGTH].occupied = 1;
        return 1;
      }
    }
  }
  return 0;
}

I2C_state_information *dequeue() {
  if (queue[queue_head].occupied) {
    I2C_state_information *ret = queue[queue_head].info;
    queue[queue_head].info = nullptr;
    queue[queue_head].occupied = 0;
    if (queue_head < I2C_QUEUE_LENGTH - 1) {
      queue_head++;
    } else {
      queue_head = 0;
    }
    return ret;
  } else
    return nullptr;
}

uint8_t i2c_handle(I2C_state_information *info) { return enqueue(info); }

volatile void i2c_thread() {
  dummy.state = Done;
  while (true) {
    if (current->state == Done) {
      I2C_state_information *next = dequeue();
      if (next != nullptr) {
        configure_i2c();
        current = next;
        start_i2c_communication();
      }
      yield();
    } else {
      yield();
    }
  }
}
