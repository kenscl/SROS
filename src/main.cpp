#include <cstdint>
#include <stdint.h>
#include "communication/LSM9DS1.h"
#include "globals.h"
#include "hw_init.h"
#include "krnl/scheduler.h"
#include "krnl/thread.h"
#include "krnl/mem.h"
#include "communication/usart.h"
#include "hal/hw_specific.h"
#include "communication/spi.h"
#include "stm32f407xx.h"

uint8_t rx[2] = {};
uint8_t tx[2] = {0, 0b00001100};
uint8_t rx2[2] = {};
uint8_t txm[2] = {0, 0b1010101};
int main (void) {
    // system config
    clock_init();
    mem_init();
    enable_usart();
    scheduler_init();
    interrupt_init();
    miscellaneous_init();

    // default run parameters
    print_welcome_msg();
    register_thread_auto(&idle_thread, 200, 0, "idle_thread");

    // User Threads are defined here
    //register_thread_auto(&LSM9DS1_thread, 2000, STD_THREAD_PRIORITY, "LSM9DS1_thread");
    //register_thread_auto(&attitude_thread, 3000, STD_THREAD_PRIORITY + 1, "attitude_thread");
    // End of user thread definitions
    SPI_init();
    uint32_t start = now_high_accuracy();
    static SPI_information info = {
      .state = Write,
      .size = 2,
      .rx_buffer = rx,
      .tx_buffer = tx,
      .target = Accelerometer,
      .adress = CTRL_REG8 
    };
    uint8_t tx2[2] = {};
    static SPI_information info2 = {
      .state = Read,
      .size = 2,
      .rx_buffer = rx2,
      .tx_buffer = tx2,
      .target = Magnetometer,
      .adress = LSM9DS1_WHO_AM_I 
    };
    static SPI_information info3 = {
      .state = Write,
      .size = 2,
      .rx_buffer = rx2,
      .tx_buffer = txm,
      .target = Magnetometer,
      .adress = CTRL_REG1_M 
    };
    static SPI_information info4 = {
      .state = Read,
      .size = 2,
      .rx_buffer = rx2,
      .tx_buffer = tx,
      .target = Magnetometer,
      .adress = CTRL_REG1_M 
    };
    uint32_t end = 0;

    print_thread_info();
    // start system
    SPI_handle(&info);
    SPI_handle(&info2);
    SPI_handle(&info3);
    SPI_handle(&info4);
    scheduler_enable();

    //idle_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
