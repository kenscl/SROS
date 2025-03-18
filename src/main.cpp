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
uint8_t tx[2] = {1, 10};
uint8_t rx2[2] = {};
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
    //register_thread_auto(&i2c_thread, 2000, STD_THREAD_PRIORITY, "i2c_thread");

    // User Threads are defined here
    //register_thread_auto(&LSM9DS1_thread, 2000, STD_THREAD_PRIORITY, "LSM9DS1_thread");
    //register_thread_auto(&attitude_thread, 3000, STD_THREAD_PRIORITY + 1, "attitude_thread");
    //register_thread_auto(&t_thred, 3000, STD_THREAD_PRIORITY, "t_thread");
    // End of user thread definitions
    SPI_init();
    uint32_t start = now_high_accuracy();
    static SPI_information info = {
      .state = Write,
      .size = 2,
      .rx_buffer = rx,
      .tx_buffer = tx,
      .target = Accelerometer,
      .adress = CTRL_REG1_G 
    };
    uint8_t tx2[2] = {};
    static SPI_information info2 = {
      .state = Read,
      .size = 2,
      .rx_buffer = rx2,
      .tx_buffer = tx2,
      .target = Accelerometer,
      .adress = CTRL_REG1_G 
    };
    uint32_t end = 0;

    print_thread_info();
    // start system
    SPI_handle(&info);
    SPI_handle(&info2);
    scheduler_enable();

    //idle_thread();
    while(1) {
        //OS_WARN("Scheduler didn't start!");
    }
}
