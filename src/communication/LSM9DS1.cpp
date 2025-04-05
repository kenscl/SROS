#include "LSM9DS1.h"
#include "../communication/usart.h"
#include "../krnl/scheduler.h"
#include "../krnl/thread.h"
#include "SPI.h"
#include <cstdint>

Vec3 gyro_bias;
/*
 * Calibration values here
 */

void LSM9DS1_calibrate_sensors() {
  gyro_bias[0] = 0.520428;
  gyro_bias[1] = 2.100893;
  gyro_bias[2] = 2.018018;
}

// data values
Vec3 LSM9DS1_gyro;
float LSM9DS1_gyro_availiable = 0;

Vec3 LSM9DS1_acc;
float LSM9DS1_acc_availiable = 0;

Vec3 LSM9DS1_mag;
float LSM9DS1_mag_availiable = 0;

Vec3 acc_bias;
Vec3 mag_bias;

uint8_t dummy_rx[2] = {};
uint8_t data[2];
void LSM9DS1_reset() {
  data[1] = 0b10000101;
  static SPI_information info = {.state = Write,
                                 .size = 2,
                                 .rx_buffer = dummy_rx,
                                 .tx_buffer = data,
                                 .target = Accelerometer,
                                 .adress = CTRL_REG8};

  while (!SPI_handle(&info))
    ;
}

uint8_t data1_g[2], data3_g[2];
void LSM9DS1_configure_gyro() {
  // general config
  uint8_t odr = (0b110 << 5); // 952 Hz
  uint8_t fs = (0b01 << 3);   // 500 dps
  uint8_t bw = (0b00 << 0);   // default bw
  data1_g[1] = odr | fs | bw;
  static SPI_information info1_g = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data1_g,
                                    .target = Accelerometer,
                                    .adress = CTRL_REG1_G};

  while (!SPI_handle(&info1_g))
    ;

  // enable highpass filter
  uint8_t hp = (0b1 << 6);
  uint8_t hpcf = (0b1001 << 0); // Frequenzy 1
  data3_g[1] = hp | hpcf;
  static SPI_information info3_g = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data3_g,
                                    .target = Accelerometer,
                                    .adress = CTRL_REG3_G};
  while (!SPI_handle(&info3_g))
    ;
}

uint8_t data6_xl[2];
uint8_t data7_xl[2];
  static SPI_information info6_xl = {.state = Write,
                                     .size = 2,
                                     .rx_buffer = dummy_rx,
                                     .tx_buffer = data6_xl,
                                     .target = Accelerometer,
                                     .adress = CTRL_REG6_XL};

void LSM9DS1_configure_accel() {
  // general config
  uint8_t odr = (0b110 << 5);    // 952 Hz
  uint8_t fs = (0b01 << 3);      // 4 g
  uint8_t bw_scale = (0b0 << 2); // bw according to odr
  uint8_t bw = (0b01 << 0);      // default bw, dosnt matter
  data6_xl[1] = odr | fs | bw;
  while (!SPI_handle(&info6_xl))
    ;
}

uint8_t data1_m[2], data2_m[2], data3_m[2], data4_m[2];
uint8_t ctrl_reg1_m = 0b11111110;
uint8_t ctrl_reg2_m = 0b00000000;
uint8_t ctrl_reg3_m =
    0b00000000; // sim needs to be 0, this is an error in the datasheet!
uint8_t ctrl_reg4_m = 0b00001100;
void LSM9DS1_configure_mag() {
  data1_m[1] = ctrl_reg1_m;
  data2_m[1] = ctrl_reg2_m;
  data3_m[1] = ctrl_reg3_m;
  data4_m[1] = ctrl_reg4_m;
  static SPI_information info1_m = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data1_m,
                                    .target = Magnetometer,
                                    .adress = CTRL_REG1_M};
  static SPI_information info2_m = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data2_m,
                                    .target = Magnetometer,
                                    .adress = CTRL_REG2_M};
  static SPI_information info3_m = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data3_m,
                                    .target = Magnetometer,
                                    .adress = CTRL_REG3_M};
  static SPI_information info4_m = {.state = Write,
                                    .size = 2,
                                    .rx_buffer = dummy_rx,
                                    .tx_buffer = data4_m,
                                    .target = Magnetometer,
                                    .adress = CTRL_REG4_M};

  while (!SPI_handle(&info1_m))
    ;
  while (!SPI_handle(&info2_m))
    ;
  while (!SPI_handle(&info3_m))
    ;
  while (!SPI_handle(&info4_m))
    ;
}

uint8_t status_reg[2] = {};
uint8_t status_reg_tx[2] = {};
uint8_t status_reg_m[2] = {};
uint8_t status_reg_m_tx[2] = {};
static SPI_information acc_and_gyro_status = {.state = Read,
                                              .size = 2,
                                              .rx_buffer = status_reg,
                                              .tx_buffer = status_reg_tx,
                                              .target = Accelerometer,
                                              .adress = LSM9DS1_STATUS_REG};

static SPI_information mag_status = {.state = Read,
                                     .size = 2,
                                     .rx_buffer = status_reg_m,
                                     .tx_buffer = status_reg_m_tx,
                                     .target = Magnetometer,
                                     .adress = LSM9DS1_STATUS_REG_M};

void LSM9DS1_read_status() {
  if (LSM9DS1_check_status())
    return; // return if message is already pending
  SPI_handle(&acc_and_gyro_status);
  SPI_handle(&mag_status);
}

uint8_t LSM9DS1_check_status() {
  if (acc_and_gyro_status.state == Done && mag_status.state == Done)
    return true;
  else
    return 0;
}

void LSM9DS1_enable_status() {
  acc_and_gyro_status.state = Read;
  mag_status.state = Read;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
void LSM9DS1_process_status() {
  if (!LSM9DS1_check_status())
    return;
  LSM9DS1_gyro_availiable = (status_reg[1] & 0b10);
  LSM9DS1_acc_availiable = (status_reg[1] & 0b1);
  LSM9DS1_mag_availiable = (status_reg_m[1] & 0b1000);
  LSM9DS1_enable_status();
}
#pragma GCC pop_options

uint8_t gyro_data[7] = {};
uint8_t gyro_data_tx[7] = {};
static SPI_information gyro_data_reg = {.state = Read,
                                        .size = 7,
                                        .rx_buffer = gyro_data,
                                        .tx_buffer = gyro_data_tx,
                                        .target = Accelerometer,
                                        .adress = OUT_X_G_L};

void LSM9DS1_read_gyro() {
  if (LSM9DS1_check_gyro())
    return;
  SPI_handle(&gyro_data_reg);
}

uint8_t LSM9DS1_check_gyro() {
  if (gyro_data_reg.state == Done)
    return true;
  else
    return false;
}

void LSM9DS1_enable_gyro() { gyro_data_reg.state = Read; }

void LSM9DS1_process_gyro() {
  if (!LSM9DS1_check_gyro())
    return;
  volatile int16_t x = (gyro_data[1 + 1] << 8) | gyro_data[0 + 1];
  volatile int16_t y = (gyro_data[3 + 1] << 8) | gyro_data[2 + 1];
  volatile int16_t z = (gyro_data[5 + 1] << 8) | gyro_data[4 + 1];
  LSM9DS1_gyro[0] = (float)(x * GYRO_SENSITIVITY) / 1000;
  LSM9DS1_gyro[1] = (float)(y * GYRO_SENSITIVITY) / 1000;
  LSM9DS1_gyro[2] = (float)(z * GYRO_SENSITIVITY) / 1000;
  LSM9DS1_gyro = LSM9DS1_gyro - gyro_bias;
  LSM9DS1_enable_gyro();
}

uint8_t acc_data[7] = {};
uint8_t acc_data_tx[7] = {};
static SPI_information acc_data_reg = {.state = Read,
                                       .size = 7,
                                       .rx_buffer = acc_data,
                                       .tx_buffer = acc_data_tx,
                                       .target = Accelerometer,
                                       .adress = OUT_X_XL_L};

void LSM9DS1_read_accel() {
  if (LSM9DS1_check_accel())
    return;
  SPI_handle(&acc_data_reg);
}

uint8_t LSM9DS1_check_accel() {
  if (acc_data_reg.state == Done)
    return true;
  else
    return false;
}

void LSM9DS1_enable_accel() { acc_data_reg.state = Read; }

void LSM9DS1_process_accel() {
  if (!LSM9DS1_check_accel())
    return;
  int16_t x = (acc_data[1 + 1] << 8) | acc_data[0 + 1];
  int16_t y = (acc_data[3 + 1] << 8) | acc_data[2 + 1];
  int16_t z = (acc_data[5 + 1] << 8) | acc_data[4 + 1];
  LSM9DS1_acc[0] = (float)(x * ACC_SENSITIVITY) / 1000;
  LSM9DS1_acc[1] = (float)(y * ACC_SENSITIVITY) / 1000;
  LSM9DS1_acc[2] = (float)(z * ACC_SENSITIVITY) / 1000;
  LSM9DS1_acc = (LSM9DS1_acc - acc_bias);
  LSM9DS1_enable_accel();
}

uint8_t mag_data[7] = {};
uint8_t mag_data_tx[7] = {};
static SPI_information mag_data_reg = {.state = Read,
                                       .size = 7,
                                       .rx_buffer = mag_data,
                                       .tx_buffer = mag_data_tx,
                                       .target = Magnetometer,
                                       .adress = OUT_X_L_M};

void LSM9DS1_read_mag() { SPI_handle(&mag_data_reg); }

uint8_t LSM9DS1_check_mag() {
  if (mag_data_reg.state == Done)
    return true;
  else
    return false;
}

void LSM9DS1_enable_mag() { mag_data_reg.state = Read; }

void LSM9DS1_process_mag() {
  if (!LSM9DS1_check_mag())
    return;
  int16_t x = (mag_data[1 + 1] << 8) | mag_data[0 + 1];
  int16_t y = (mag_data[3 + 1] << 8) | mag_data[2 + 1];
  int16_t z = (mag_data[5 + 1] << 8) | mag_data[4 + 1];
  LSM9DS1_mag[0] = (float)(x * MAG_SENSITIVITY) / 1000;
  LSM9DS1_mag[1] = (float)(y * MAG_SENSITIVITY) / 1000;
  LSM9DS1_mag[2] = (float)(z * MAG_SENSITIVITY) / 1000;
  LSM9DS1_mag = LSM9DS1_mag - mag_bias;
  LSM9DS1_enable_mag();
}

uint8_t who_data[2], who_data_m[2] = {};
uint8_t who_data_tx[2], who_data_tx_m[2] = {};
static SPI_information who_data_reg = {.state = Read,
                                       .size = 2,
                                       .rx_buffer = who_data,
                                       .tx_buffer = who_data_tx,
                                       .target = Accelerometer,
                                       .adress = LSM9DS1_WHO_AM_I};

static SPI_information who_data_reg_m = {.state = Read,
                                         .size = 2,
                                         .rx_buffer = who_data_m,
                                         .tx_buffer = who_data_tx_m,
                                         .target = Magnetometer,
                                         .adress = LSM9DS1_WHO_AM_I};

void LSM9DS1_read_WHO_AM_I() {
  if (who_data_reg.state == Done)
    return;
  if (who_data_reg_m.state == Done)
    return;
  SPI_handle(&who_data_reg);
  SPI_handle(&who_data_reg_m);
}

uint8_t LSM9DS1_check_WHO_AM_I() {
  if (who_data_reg.state == Done && who_data_reg_m.state == Done)
    return true;
  else
    return false;
}

void LSM9DS1_enable_WHO_AM_I() {
  who_data_reg.state = Read;
  who_data_reg_m.state = Read;
}

void LSM9DS1_process_WHO_AM_I() {
  if (!LSM9DS1_check_WHO_AM_I())
    return;
  if (who_data[1] == 104) {
  } else {
    os_printf("SPI error gyro! \n");
  }
  if (who_data_m[1] == 61) {
  } else {
    os_printf("SPI error mag! %d \n", who_data_m[1]);
  }
  LSM9DS1_enable_WHO_AM_I();
}

static SPI_information mag_data_tst = {.state = Read,
                                       .size = 1,
                                       .rx_buffer = acc_data,
                                       .tx_buffer = acc_data_tx,
                                       .target = Magnetometer,
                                       .adress = OUT_X_L_M};

uint8_t eq_cnt = 0;
uint32_t last_time = 0;
volatile void LSM9DS1_thread() {
  SPI_init();
  LSM9DS1_reset();
  sleep(10 * MILLISECONDS);
  LSM9DS1_configure_gyro();
  LSM9DS1_configure_accel();
  LSM9DS1_configure_mag();
  LSM9DS1_read_WHO_AM_I();
  // calibration
  //LSM9DS1_calibrate_sensors();
  sleep(10 * MILLISECONDS);
  LSM9DS1_process_WHO_AM_I();
  while (1) {
    volatile uint32_t next_time = now() + 3 * MILLISECONDS;
    last_time = now();
    os_printf("LSM9DS1_gyro, ");
    LSM9DS1_gyro.print_bare();
     LSM9DS1_read_gyro();
    os_printf("LSM9DS1_mag, ");
    LSM9DS1_mag.print_bare();
    LSM9DS1_read_mag();
    os_printf("LSM9DS1_acc, ");
    LSM9DS1_acc.print_bare();
    LSM9DS1_read_accel();

    SPI_send_next();
    sleep(2 * MILLISECONDS);
    Vec3 last_gyro = LSM9DS1_gyro;
    LSM9DS1_process_gyro();
    LSM9DS1_process_accel();
    LSM9DS1_process_mag();
    if (last_gyro == LSM9DS1_gyro) {
      eq_cnt++;
    } else {
      eq_cnt = 0;
    }

    sleep_until(next_time);
    if (eq_cnt > 10) {
      eq_cnt = 0;
      os_printf("\n\n\nconnection error! \n\n\n");
      SPI_init();
      sleep(1 * MILLISECONDS);
      LSM9DS1_reset();
      SPI_send_next();
      sleep(1 * MILLISECONDS);

      LSM9DS1_configure_gyro();
      LSM9DS1_configure_accel();
      LSM9DS1_configure_mag();
      LSM9DS1_read_WHO_AM_I();
      SPI_send_next();
      sleep(20 * MILLISECONDS);

      LSM9DS1_process_WHO_AM_I();

      LSM9DS1_enable_status();
      LSM9DS1_enable_gyro();
      LSM9DS1_enable_mag();
      LSM9DS1_enable_accel();

      next_time = now() + 3 * MILLISECONDS;
      last_time = now();
    }
  }
}
