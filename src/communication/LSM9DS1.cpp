#include "LSM9DS1.h"
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "../krnl/thread.h"
#include "../krnl/scheduler.h"
#include "i2c.h"
#include "stm32f407xx.h"
#include "../hal/hw_specific.h"

#define SPI_QUEUE_LENGTH 20
SPI_information * queue[SPI_QUEUE_LENGTH] = {};
SPI_information * current = 0;
uint16_t queue_head = 0;
SPI_information * empty;
uint8_t dma_done = 1;
uint8_t enqueue(SPI_information *info) {
  for (int i = queue_head; i < SPI_QUEUE_LENGTH + queue_head; ++i) {
    if (i < SPI_QUEUE_LENGTH) {
      if (queue[i]->size == 0) {
        queue[i] = info;
        return 1;
      }
    } else {
      if (queue[i - SPI_QUEUE_LENGTH]->size == 0) {
        queue[i - SPI_QUEUE_LENGTH] = info;
        return 1;
      }
    }
  }
  return 0;
}

SPI_information *dequeue() {
  if (queue[queue_head]->size != 0) {
    SPI_information *ret = queue[queue_head];
    queue[queue_head] = empty;
    if (queue_head < SPI_QUEUE_LENGTH - 1) {
      queue_head++;
    } else {
      queue_head = 0;
    }
    return ret;
  } else
    return nullptr;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
void SPI_send_next() {
    if (queue[queue_head]->size != 0) {
      if (current->state == Done) {
	   current = dequeue();

	   if (current->state == Done) {
	     current = empty;
	     return;
	   }
	   if (current->state == Read) {
	     if (current->target == Accelerometer)
	       LSM9DS1_A_read_register_dma(current->adress, current->rx_buffer,
					   current->tx_buffer, current->size);
	     if (current->target == Magnetometer)
	       LSM9DS1_M_read_register_dma(current->adress, current->rx_buffer,
					   current->tx_buffer, current->size);
	   } else {
	     if (current->target == Accelerometer)
	       LSM9DS1_A_write_register_dma(current->adress, current->rx_buffer,
					    current->tx_buffer);
	     if (current->target == Magnetometer)
	       LSM9DS1_M_write_register_dma(current->adress, current->rx_buffer,
					    current->tx_buffer);
	   }
      }
    } else {
	if (current->state == Done) {
	    current = empty;
	    return;
	}
    }
}
#pragma GCC pop_options

volatile void SPI_thread() {
    while (1) {
      os_interrupt_disable();
      volatile uint32_t state = DMA2->LISR;
      volatile uint32_t state2 = DMA2->HISR;
      if (dma_done)
	SPI_send_next();
      os_interrupt_enable();
      yield();
    }
}

// SPI
uint8_t SPI_handle(SPI_information *info) {
  return enqueue(info);
}

void CS_A_H() {
    GPIOA->BSRR = GPIO_BSRR_BS_4;
}

void CS_A_L() {
    GPIOA->BSRR = GPIO_BSRR_BR_4;
}


void CS_M_H() {
    GPIOA->BSRR = GPIO_BSRR_BS_1;
}

void CS_M_L() {
    GPIOA->BSRR = GPIO_BSRR_BR_1;
}

void DMA2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // tx
    DMA2_Stream3->CR = 0;
    DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream3->CR |= (0b011 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream3->CR |= (0b01 << DMA_SxCR_DIR_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_PL_Pos);

    //rx 
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream0->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream0->CR &= ~(0b011 << DMA_SxCR_DIR_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_PL_Pos);
    DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

    //NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    //NVIC_SetPriority(DMA2_Stream3_IRQn, 11);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 10);
}

static SPI_information dmy = {
    .state = Done,
    .size = 0,
    .rx_buffer = nullptr,
    .tx_buffer = nullptr,
    .target = Accelerometer,
    .adress = 0
};

void SPI_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 

    GPIOA->MODER |= 0b01 << (4 * 2); // general purpose output 
    GPIOA->MODER |= 0b01 << (1 * 2); // general purpose output 
    CS_A_H();
    CS_M_H();
    
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

    // dummy
    empty = &dmy;
    current = &dmy;
    for (int i = 0; i < SPI_QUEUE_LENGTH; ++i) {
      queue[i] = &dmy;
    }
    
    DMA2_init();

    register_thread_auto(&SPI_thread, 500, 10, "SPI_thread");
}


uint8_t LSM9DS1_A_write_register_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer) {
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    uint8_t dma_done = 0;

    dma_tx_buffer[0] = reg & 0x7F;
    DMA2_Stream3->M0AR = (uint32_t)dma_tx_buffer;
    DMA2_Stream3->NDTR = 2;

    DMA2_Stream0->M0AR = (uint32_t) dma_rx_buffer;
    DMA2_Stream0->NDTR = 2;

    CS_A_L();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return 0;
}

uint8_t LSM9DS1_A_read_register_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer, size_t size) {
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    uint8_t dma_done = 0;

    dma_tx_buffer[0] = reg | 0x80;
    for (int i = 1; i < size; ++i)
      dma_tx_buffer[i] = 0x00;
    DMA2_Stream3->M0AR = (uint32_t) dma_tx_buffer;
    DMA2_Stream3->NDTR = size;

    DMA2_Stream0->M0AR = (uint32_t) dma_rx_buffer;
    DMA2_Stream0->NDTR = size;

    CS_A_L();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return 0;
}

uint8_t LSM9DS1_M_read_register_dma(uint8_t reg, uint8_t *dma_rx_buffer,
                                    uint8_t *dma_tx_buffer, size_t size) {
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    uint8_t dma_done = 0;

    dma_tx_buffer[0] = reg | 0x80;
    if (size > 2)
      dma_tx_buffer[0] |= (1 << 6);
    for (int i = 1; i < size; ++i)
      dma_tx_buffer[i] = 0x00;
    DMA2_Stream3->M0AR = (uint32_t)dma_tx_buffer;
    DMA2_Stream3->NDTR = size;

    DMA2_Stream0->M0AR = (uint32_t) dma_rx_buffer;
    DMA2_Stream0->NDTR = size;

    CS_M_L();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return 0;
}

uint8_t LSM9DS1_M_write_register_dma(uint8_t reg, uint8_t * dma_rx_buffer, uint8_t * dma_tx_buffer) {
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    uint8_t dma_done = 0;

    dma_tx_buffer[0] = reg & 0x7F;
    DMA2_Stream3->M0AR = (uint32_t)dma_tx_buffer;
    DMA2_Stream3->NDTR = 2;

    DMA2_Stream0->M0AR = (uint32_t) dma_rx_buffer;
    DMA2_Stream0->NDTR = 2;

    CS_M_L();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return 0;
}

extern "C" {
    void dma2_stream3_handler(){
	if (DMA2->LISR & DMA_LISR_TCIF3) {
	    DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
	}
	if (DMA2->LISR & DMA_LISR_HTIF3) {
	    DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
	}
    }

    void dma2_stream0_handler() {
      volatile uint32_t state = DMA2->LISR;
	if (DMA2->LISR & DMA_LISR_TCIF0) {
	    DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
	    DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
	    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
	    DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
	    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	    CS_A_H();
	    CS_M_H();
	    current->state = Done;
	    current = empty;
	    //os_printf("res: %d \n", current->rx_buffer[1]);
	    dma_done = 1;
            SPI_send_next();
            }
    }
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

uint8_t dummy_rx[2] = {};
uint8_t data[2];
void LSM9DS1_reset() {
    data[1] = 0b10000101;
    static SPI_information info = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data,
	.target = Accelerometer,
	.adress = CTRL_REG8
    };

    while (!SPI_handle(&info));
}

uint8_t data1_g[2], data3_g[2];
void LSM9DS1_configure_gyro() {
    // general config
    uint8_t odr = (0b110 << 5); // 476 Hz
    uint8_t fs = (0b01 << 3); // 500 dps
    uint8_t bw = (0b00 << 0); // default bw
    data1_g[1] = odr | fs | bw;
    static SPI_information info1_g = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data1_g,
	.target = Accelerometer,
	.adress = CTRL_REG1_G
    };

    while (!SPI_handle(&info1_g));

    // enable highpass filter
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b1001<< 0); // Frequenzy 1 
    data3_g[1] = hp | hpcf ;
    static SPI_information info3_g = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data3_g,
	.target = Accelerometer,
	.adress = CTRL_REG3_G
    };
    while (!SPI_handle(&info3_g));
}

uint8_t data6_xl[2];
void LSM9DS1_configure_accel() {
    // general config
    uint8_t odr = (0b100 << 5); // 952 Hz
    uint8_t fs = (0b00 << 3); // 2 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b01 << 0); // default bw, dosnt matter
    data6_xl[1] = odr | fs | bw;
    static SPI_information info6_xl = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data6_xl,
	.target = Accelerometer,
	.adress = CTRL_REG6_XL
    };

    while (!SPI_handle(&info6_xl));
}

uint8_t data1_m[2], data2_m[2], data3_m[2], data4_m[2];
uint8_t ctrl_reg1_m = 0b11111100;
uint8_t ctrl_reg2_m = 0b00000000;
uint8_t ctrl_reg3_m = 0b00000000; // sim needs to be 0, this is an error in the datasheet!
uint8_t ctrl_reg4_m = 0b00001100;
void LSM9DS1_configure_mag() {
    data1_m[1] = ctrl_reg1_m;
    data2_m[1] = ctrl_reg2_m;
    data3_m[1] = ctrl_reg3_m;
    data4_m[1] = ctrl_reg4_m;
    static SPI_information info1_m = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data1_m,
	.target = Magnetometer,
	.adress = CTRL_REG1_M
    };
    static SPI_information info2_m = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data2_m,
	.target = Magnetometer,
	.adress = CTRL_REG2_M
    };
    static SPI_information info3_m = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data3_m,
	.target = Magnetometer,
	.adress = CTRL_REG3_M
    };
    static SPI_information info4_m = {
        .state = Write,
	.size = 2,
	.rx_buffer = dummy_rx,
	.tx_buffer = data4_m,
	.target = Magnetometer,
	.adress = CTRL_REG4_M
    };

    while (!SPI_handle(&info1_m));
    while (!SPI_handle(&info2_m));
    while (!SPI_handle(&info3_m));
    while (!SPI_handle(&info4_m));
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

uint8_t status_reg[2] = {};
uint8_t status_reg_tx[2] = {};
uint8_t status_reg_m[2] = {};
uint8_t status_reg_m_tx[2] = {};
static SPI_information acc_and_gyro_status = {
    .state = Read,
    .size = 2,
    .rx_buffer = status_reg,
    .tx_buffer = status_reg_tx,
    .target = Accelerometer,
    .adress = LSM9DS1_STATUS_REG 
};

static SPI_information mag_status = {
    .state = Read,
    .size = 2,
    .rx_buffer = status_reg_m,
    .tx_buffer = status_reg_m_tx,
    .target = Magnetometer,
    .adress = LSM9DS1_STATUS_REG_M
};

void LSM9DS1_read_status() {
    if (LSM9DS1_check_status()) return; // return if message is already pending
    SPI_handle(&acc_and_gyro_status);
    SPI_handle(&mag_status);
}

uint8_t LSM9DS1_check_status() {
    if (acc_and_gyro_status.state == Done && mag_status.state == Done) return true;
    else return 0;
}

void LSM9DS1_enable_status() {
    acc_and_gyro_status.state = Read;
    mag_status.state = Read;
}

void LSM9DS1_process_status() {
    if (!LSM9DS1_check_status()) return;
    LSM9DS1_gyro_availiable = (status_reg[1] & 0b10);
    LSM9DS1_acc_availiable = (status_reg[1] & 0b1);
    LSM9DS1_mag_availiable = (status_reg_m[1] & 0b1000);
    LSM9DS1_enable_status();
}

uint8_t gyro_data[7] = {};
uint8_t gyro_data_tx[7] = {};
static SPI_information gyro_data_reg = {
    .state = Read,
    .size = 7,
    .rx_buffer = gyro_data,
    .tx_buffer = gyro_data_tx,
    .target = Accelerometer,
    .adress = OUT_X_G_L 
};

void LSM9DS1_read_gyro() {
    if (LSM9DS1_check_gyro()) return;
    SPI_handle(&gyro_data_reg);
}

uint8_t LSM9DS1_check_gyro() {
    if (gyro_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_gyro() {
    gyro_data_reg.state = Read;
}

void LSM9DS1_process_gyro() {
    if (!LSM9DS1_check_gyro()) return;
    volatile int16_t x = (gyro_data[1+1] << 8) | gyro_data[0+1];
    volatile int16_t y = (gyro_data[3+1] << 8) | gyro_data[2+1];
    volatile int16_t z = (gyro_data[5+1] << 8) | gyro_data[4+1];
    LSM9DS1_gyro[0] = (float) (x * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[1] = (float) (y * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[2] = (float) (z * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro = LSM9DS1_gyro - gyro_bias;
    LSM9DS1_enable_gyro();
}

uint8_t acc_data[7] = {};
uint8_t acc_data_tx[7] = {};
static SPI_information acc_data_reg = {
    .state = Read,
    .size = 7,
    .rx_buffer = acc_data,
    .tx_buffer = acc_data_tx,
    .target = Accelerometer,
    .adress = OUT_X_XL_L 
};

void LSM9DS1_read_accel() {
    if (LSM9DS1_check_accel()) return; 
    SPI_handle(&acc_data_reg);
}

uint8_t LSM9DS1_check_accel() {
    if (acc_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_accel() {
    acc_data_reg.state = Read;
}

void LSM9DS1_process_accel() {
    if (!LSM9DS1_check_accel()) return; 
    int16_t x = (acc_data[1+1] << 8) | acc_data[0+1];
    int16_t y = (acc_data[3+1] << 8) | acc_data[2+1];
    int16_t z = (acc_data[5+1] << 8) | acc_data[4+1];
    LSM9DS1_acc[0] = (float) (x * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[1] = (float) (y * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[2] = (float) (z * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc = (LSM9DS1_acc - acc_bias) * - 1;
    LSM9DS1_enable_accel();
}

uint8_t mag_data[7] = {};
uint8_t mag_data_tx[7] = {};
static SPI_information mag_data_reg = {
    .state = Read,
    .size = 7,
    .rx_buffer = mag_data,
    .tx_buffer = mag_data_tx,
    .target = Magnetometer,
    .adress = OUT_X_L_M
};

void LSM9DS1_read_mag() {
    SPI_handle(&mag_data_reg);
}

uint8_t LSM9DS1_check_mag() {
    if (mag_data_reg.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_mag() {
    mag_data_reg.state = Read;
}

void LSM9DS1_process_mag() {
    if (!LSM9DS1_check_mag()) return;
    int16_t x = (mag_data[1+1] << 8) | mag_data[0+1];
    int16_t y = (mag_data[3+1] << 8) | mag_data[2+1];
    int16_t z = (mag_data[5+1] << 8) | mag_data[4+1];
    LSM9DS1_mag[0] = (float) (x * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[1] = (float) (y * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[2] = (float) (z * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag = LSM9DS1_mag - mag_bias;
    LSM9DS1_enable_mag();
}

uint8_t who_data[2], who_data_m[2] = {};
uint8_t who_data_tx[2],  who_data_tx_m[2] = {};
static SPI_information who_data_reg = {
    .state = Read,
    .size = 2,
    .rx_buffer = who_data,
    .tx_buffer = who_data_tx,
    .target = Accelerometer,
    .adress = LSM9DS1_WHO_AM_I 
};

static SPI_information who_data_reg_m = {
    .state = Read,
    .size = 2,
    .rx_buffer = who_data_m,
    .tx_buffer = who_data_tx_m,
    .target = Magnetometer,
    .adress = LSM9DS1_WHO_AM_I 
};

void LSM9DS1_read_WHO_AM_I() {
  if (who_data_reg.state == Done) return;
  if (who_data_reg_m.state == Done) return;
  SPI_handle(&who_data_reg);
  SPI_handle(&who_data_reg_m);
}

uint8_t LSM9DS1_check_WHO_AM_I() {
    if (who_data_reg.state == Done && who_data_reg_m.state == Done) return true;
    else return false;
}

void LSM9DS1_enable_WHO_AM_I() {
    who_data_reg.state = Read;
    who_data_reg_m.state = Read;
}

void LSM9DS1_process_WHO_AM_I() {
    if (!LSM9DS1_check_WHO_AM_I()) return;
    if (who_data[1] == 104) {}
    else {
      os_printf("SPI error gyro! \n");
    }
    if (who_data_m[1] == 61) {}
    else {
      os_printf("SPI error mag! %d \n", who_data_m[1]);
    }
    LSM9DS1_enable_WHO_AM_I();
}

static SPI_information mag_data_tst= {
    .state = Read,
    .size = 1,
    .rx_buffer = acc_data,
    .tx_buffer = acc_data_tx,
    .target = Magnetometer,
    .adress = OUT_X_L_M
};
volatile void LSM9DS1_thread() {
  uint8_t tst = now();
  LSM9DS1_reset();
  sleep(10 * MILLISECONDS);
  LSM9DS1_configure_gyro();
  LSM9DS1_configure_accel();
  LSM9DS1_configure_mag();
  LSM9DS1_read_WHO_AM_I();
  // calibration
  LSM9DS1_calibrate_sensors();
  sleep(10 * MILLISECONDS);
  LSM9DS1_process_WHO_AM_I();
  while (1) {
    LSM9DS1_read_status();
    volatile uint32_t next_time = now() + 3 * MILLISECONDS;
    yield();
    LSM9DS1_process_status();
    if (LSM9DS1_gyro_availiable) {
      os_printf("LSM9DS1_gyro, ");
      LSM9DS1_gyro.print_bare();
      LSM9DS1_read_gyro();
    }
    if (LSM9DS1_mag_availiable) {
      os_printf("LSM9DS1_mag, ");
      LSM9DS1_mag.print_bare();
      LSM9DS1_read_mag();
      }
    if (LSM9DS1_acc_availiable) {
      os_printf("LSM9DS1_acc, ");
      LSM9DS1_acc.print_bare();
      LSM9DS1_read_accel();
    }
    SPI_send_next();
    sleep(2 * MILLISECONDS);
    LSM9DS1_process_gyro();
    LSM9DS1_process_accel();
    LSM9DS1_process_mag();
    sleep_until(next_time);
  }
}
