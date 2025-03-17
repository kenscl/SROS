#include "LSM9DS1.h"
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "../krnl/thread.h"
#include "../krnl/scheduler.h"
#include "i2c.h"
#include "stm32f407xx.h"

// SPI

void CS_A_H() {
    GPIOA->BSRR = GPIO_BSRR_BS_4;
}

void CS_A_L() {
    GPIOA->BSRR = GPIO_BSRR_BR_4;
}

void DMA2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // tx
    DMA2_Stream3->CR = 0;
    DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream3->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream3->CR |= (0b01 << DMA_SxCR_DIR_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream3->CR |= (3 << DMA_SxCR_PL_Pos);

    //rx 
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream0->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream0->CR &= ~(0b11 << DMA_SxCR_DIR_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream0->CR |= (3 << DMA_SxCR_PL_Pos);

    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void SPI_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 

    GPIOA->MODER |= 0b01 << (4 * 2); // general purpose output 
    CS_A_H();
    
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2)); // af mode
    GPIOA->OSPEEDR |= (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)); // High speed
    GPIOA->AFR[0]  |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4)); // af 5
    SPI1->CR1 = 0;

    SPI1->CR1 = SPI_CR1_MSTR     // Master 
              | SPI_CR1_SSM      // Software CS management
              | SPI_CR1_SSI      // Set nss high
              | (3 << 3);    // fPCLK/8 

    SPI1->CR1 |= SPI_CR1_SPE;  
    SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN; 
    DMA2_init();
}

uint8_t SPI_A_transmit(uint8_t byte) {
    while (!(SPI1->SR & SPI_SR_TXE)) {} // Wait for TXE
    *(volatile uint8_t *)&SPI1->DR = byte;
    while (!(SPI1->SR & SPI_SR_RXNE)) {} // Wait for RXNE
    return *(volatile uint8_t *)&SPI1->DR;
}

void LSM9DS1_A_write(uint8_t reg, uint8_t byte) {
    CS_A_L();
    SPI_A_transmit(reg & 0x7F);
    SPI_A_transmit(byte);
    CS_A_H();
}

uint8_t LSM9DS1_A_read_register(uint8_t reg) {
    CS_A_L();
    SPI_A_transmit(reg | 0x80);
    uint8_t ret = SPI_A_transmit(0x00); // dummy
    CS_A_H();
    return ret;
}

uint8_t dma_transfere_complete = 1;
uint8_t LSM9DS1_A_read_register_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer) {
    dma_transfere_complete = 0;
    dma_tx_buffer[0] = reg | 0x80;
    dma_tx_buffer[1] = 0x00;

    DMA2_Stream3->M0AR |= (uint32_t) dma_tx_buffer;
    DMA2_Stream3->NDTR |= 2;
    
    DMA2_Stream0->M0AR |= (uint32_t) dma_rx_buffer;
    DMA2_Stream0->NDTR |= 2;

    CS_A_L();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return 0;
}

extern "C" {
    void dma2_stream3_handler(){
	if (DMA2->LISR & DMA_LISR_TCIF3) {
	    DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
	}
    }

    void dma2_stream0_handler() {
	if (DMA2->LISR & DMA_LISR_TCIF0) {
	    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
	    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	    CS_A_H();
	    dma_transfere_complete = 1;
	}
    }
}

uint8_t * LSM9DS1_A_read_register_multi(uint8_t reg, uint8_t * data, size_t size) {
    CS_A_L();
    SPI_A_transmit(reg | 0x80);
    for (int i = 0; i < size; ++i) {
        data[i] = SPI_A_transmit(0x00); // dummy
    }
    CS_A_H();
    return data;
}

// data values
Vec3 LSM9DS1_gyro;
float LSM9DS1_gyro_availiable = 0;

Vec3 LSM9DS1_acc;
float LSM9DS1_acc_availiable = 0;

Vec3 LSM9DS1_mag;
float LSM9DS1_mag_availiable = 0;

Vec3 acc_bias;
Vec3 gyro_bias;
Vec3 mag_bias;

void LSM9DS1_reset() {
    uint8_t data[1];
    data[0] = 0b0000001;
    static I2C_state_information info = {
        .state = Sending,
        .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
        .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
        .device_subadress = CTRL_REG8,
        .recieve_bytes = 1,
        .data = data
    };

    while (!i2c_handle(&info));
}

void LSM9DS1_configure_gyro() {
    // general config
    uint8_t odr = (0b110 << 5); // 476 Hz
    uint8_t fs = (0b01 << 3); // 500 dps
    uint8_t bw = (0b00 << 0); // default bw
    
    uint8_t data1_g[1], data3_g[1];
    data1_g[0] = odr | fs | bw;
    static I2C_state_information info1_g = {
        .state = Sending,
        .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
        .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
        .device_subadress = CTRL_REG1_G,
        .recieve_bytes = 1,
        .data = data1_g
    };

    while (!i2c_handle(&info1_g));

    // enable highpass filter
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b1001<< 0); // Frequenzy 1 
    data3_g[0] = hp | hpcf ;
    static I2C_state_information info3_g = {
        .state = Sending,
        .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
        .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
        .device_subadress = CTRL_REG3_G,
        .recieve_bytes = 1,
        .data = data1_g
    };
    while (!i2c_handle(&info3_g));
}

void LSM9DS1_configure_accel() {
    // general config
    uint8_t odr = (0b100 << 5); // 952 Hz
    uint8_t fs = (0b00 << 3); // 2 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b01 << 0); // default bw, dosnt matter
    uint8_t data6_xl[1];
    data6_xl[0] = odr | fs | bw;
    static I2C_state_information info6_xl = {
        .state = Sending,
        .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
        .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
        .device_subadress = CTRL_REG6_XL,
        .recieve_bytes = 1,
        .data = data6_xl
    };

    while (!i2c_handle(&info6_xl));
}

void LSM9DS1_configure_mag() {
    uint8_t ctrl_reg1_m = 0b11111100;
    uint8_t ctrl_reg2_m = 0b00000000;
    uint8_t ctrl_reg3_m = 0b00000000;
    uint8_t ctrl_reg4_m = 0b00001100;
    uint8_t data1_m[1], data2_m[1], data3_m[1], data4_m[1];
    data1_m[0] = ctrl_reg1_m;
    data2_m[0] = ctrl_reg2_m;
    data3_m[0] = ctrl_reg3_m;
    data4_m[0] = ctrl_reg4_m;
    static I2C_state_information info1_m = {
        .state = Sending,
        .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
        .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
        .device_subadress = CTRL_REG1_M,
        .recieve_bytes = 1,
        .data = data1_m 
    };
    static I2C_state_information info2_m = {
        .state = Sending,
        .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
        .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
        .device_subadress = CTRL_REG2_M,
        .recieve_bytes = 1,
        .data = data2_m 
    };
    static I2C_state_information info3_m = {
        .state = Sending,
        .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
        .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
        .device_subadress = CTRL_REG3_M,
        .recieve_bytes = 1,
        .data = data3_m 
    };
    static I2C_state_information info4_m = {
        .state = Sending,
        .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
        .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
        .device_subadress = CTRL_REG4_M,
        .recieve_bytes = 1,
        .data = data4_m 
    };

    while (!i2c_handle(&info1_m));
    while (!i2c_handle(&info2_m));
    while (!i2c_handle(&info3_m));
    while (!i2c_handle(&info4_m));
}

void LSM9DS1_calibrate_sensors() {
    gyro_bias = gyro_bias * 0;
    acc_bias = acc_bias * 0;
    mag_bias = mag_bias * 0;

    Vec3 temp_gyro;
    Vec3 temp_acc;
    Vec3 temp_mag;

    float mag_x_max = 0.83, mag_x_min = -0.101;
    float mag_y_max = 0.86, mag_y_min = -0.079;
    float mag_z_max = -0.707, mag_z_min = -1.58;
    mag_bias[0] = (mag_x_max + mag_x_min) / 2;
    mag_bias[1] = (mag_x_max + mag_x_min) / 2;
    mag_bias[2] = (mag_z_max + mag_z_min) / 2;
}

uint8_t status_reg[1] = {};
static I2C_state_information acc_and_gyro_status_reg = {
    .state = Recieving,
    .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
    .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
    .device_subadress = LSM9DS1_STATUS_REG,
    .recieve_bytes = 1,
    .data = status_reg 
};

uint8_t status_reg_m[1] = {};
static I2C_state_information mag_status_reg = {
    .state = Recieving,
    .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
    .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
    .device_subadress = LSM9DS1_STATUS_REG_M,
    .recieve_bytes = 1,
    .data = status_reg_m 
};

void LSM9DS1_read_status() {
    //if (LSM9DS1_check_status()) return; // return if message is already pending
    i2c_handle(&acc_and_gyro_status_reg);
    //i2c_handle(&mag_status_reg);
}

uint8_t LSM9DS1_check_status() {
    if (acc_and_gyro_status_reg.state == Done && mag_status_reg.state == Done) return true;
    else return 0;
}

void LSM9DS1_enable_status() {
    acc_and_gyro_status_reg.state = Recieving;
    mag_status_reg.state = Recieving;
}

void LSM9DS1_process_status() {
    if (!LSM9DS1_check_status()) return;
    LSM9DS1_gyro_availiable = (status_reg[0] & 0b10);
    LSM9DS1_acc_availiable = (status_reg[0] & 0b1);
    LSM9DS1_mag_availiable = (status_reg_m[0] & 0b1000);
    LSM9DS1_enable_status();
}

uint8_t gyro_data[6] = {};
static I2C_state_information gyro_data_reg = {
    .state = Recieving,
    .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
    .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
    .device_subadress = OUT_X_G_L,
    .recieve_bytes = 0,
    .data = gyro_data 
};

void LSM9DS1_read_gyro() {
    if (LSM9DS1_check_gyro()) return;
    i2c_handle(&gyro_data_reg);
}

uint8_t LSM9DS1_check_gyro() {
    if (gyro_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_gyro() {
    gyro_data_reg.state = Recieving;
}

void LSM9DS1_process_gyro() {
    if (!LSM9DS1_check_gyro()) return;
    int16_t x = (gyro_data[1] << 8) | gyro_data[0];
    int16_t y = (gyro_data[3] << 8) | gyro_data[2];
    int16_t z = (gyro_data[5] << 8) | gyro_data[4];
    LSM9DS1_gyro[0] = (float) (x * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[1] = (float) (y * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[2] = (float) (z * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro = LSM9DS1_gyro - gyro_bias;
    LSM9DS1_enable_gyro();
}

uint8_t acc_data[6] = {};
static I2C_state_information acc_data_reg = {
    .state = Recieving,
    .device_adress_write = LSM9DS1_ACC_AND_GYRO_WRITE,
    .device_adress_recieve = LSM9DS1_ACC_AND_GYRO_READ,
    .device_subadress = OUT_X_XL_L,
    .recieve_bytes = 6,
    .data = acc_data 
};

void LSM9DS1_read_accel() {
    if (LSM9DS1_check_accel()) return; 
    i2c_handle(&acc_data_reg);
}

uint8_t LSM9DS1_check_accel() {
    if (acc_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_accel() {
    acc_data_reg.state = Recieving;
}

void LSM9DS1_process_accel() {
    if (!LSM9DS1_check_accel()) return; 
    int16_t x = (acc_data[1] << 8) | acc_data[0];
    int16_t y = (acc_data[3] << 8) | acc_data[2];
    int16_t z = (acc_data[5] << 8) | acc_data[4];
    LSM9DS1_acc[0] = (float) (x * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[1] = (float) (y * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[2] = (float) (z * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc = (LSM9DS1_acc - acc_bias) * - 1;
    LSM9DS1_enable_gyro();
}

uint8_t mag_data[6] = {};
static I2C_state_information mag_data_reg = {
    .state = Recieving,
    .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
    .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
    .device_subadress = OUT_X_L_M,
    .recieve_bytes = 6,
    .data = mag_data 
};

void LSM9DS1_read_mag() {
    i2c_handle(&mag_data_reg);
}

uint8_t LSM9DS1_check_mag() {
    if (mag_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_mag() {
    acc_data_reg.state = Recieving;
}

void LSM9DS1_process_mag() {
    if (!LSM9DS1_check_mag()) return;
    int16_t x = (mag_data[1] << 8) | mag_data[0];
    int16_t y = (mag_data[3] << 8) | mag_data[2];
    int16_t z = (mag_data[5] << 8) | mag_data[4];
    LSM9DS1_mag[0] = (float) (x * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[1] = (float) (y * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[2] = (float) (z * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag = LSM9DS1_mag - mag_bias;
    LSM9DS1_enable_mag();
}

uint8_t _data[1] = {};
volatile void LSM9DS1_thread() {
  uint8_t tst = now();
  //LSM9DS1_reset();
  //LSM9DS1_configure_gyro();
  //LSM9DS1_configure_accel();
  LSM9DS1_configure_mag();
  // calibration
  //LSM9DS1_read_gyro();
  LSM9DS1_calibrate_sensors();
  sleep(10 * MILLISECONDS);
  //LSM9DS1_read_status();

  static I2C_state_information _data_reg = {
      .state = Recieving,
      .device_adress_write = LSM9DS1_MAG_SENSOE_WRITE,
      .device_adress_recieve = LSM9DS1_MAG_SENSOE_READ,
      .device_subadress = CTRL_REG1_M,
      .recieve_bytes = 1,
      .data = _data};
  i2c_handle(&_data_reg);
  // LSM9DS1_read_gyro();
  while (1) {
      volatile uint32_t next_time = now() + 3 * MILLISECONDS;
      //LSM9DS1_read_status();
      sleep(1 * MILLISECONDS);
      //LSM9DS1_process_mag();
      //LSM9DS1_process_accel();
      //LSM9DS1_process_gyro();
      //LSM9DS1_process_status();
      //LSM9DS1_gyro.print();
      //if (LSM9DS1_gyro_availiable) {
      //    LSM9DS1_read_gyro();
      //}
      //if (LSM9DS1_acc_availiable) {
      //    LSM9DS1_read_accel();
      //}
      //if (LSM9DS1_mag_availiable) {
      //    LSM9DS1_read_mag();
      //}
      sleep_until(next_time);
  }
}
