#include "LSM9DS1.h"
#include <cstdint>
#include "../krnl/mem.h"
#include "../communication/usart.h"
#include "../krnl/thread.h"
#include "../krnl/scheduler.h"
#include "../ekf/ekf.h"

void LSM9DS1_write_acc_and_gyro_register(uint8_t reg, uint8_t data) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // data
    i2c_send_data(data);
    // sp
    i2c_stop();
}

uint8_t * LSM9DS1_read_acc_and_gyro_register(uint8_t reg) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_ACC_AND_GYRO_READ, 1);
    return result;
}


uint8_t * LSM9DS1_read_acc_and_gyro_multi(uint8_t reg, uint16_t size) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_ACC_AND_GYRO_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_ACC_AND_GYRO_READ, size);
    return result;
}

void LSM9DS1_write_mag_register(uint8_t reg, uint8_t data) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_MAG_SENSOE_WRITE);
    // SUB
    i2c_send_data(reg);
    // data
    i2c_send_data(data);
    // sp
    i2c_stop();
}

uint8_t * LSM9DS1_read_mag_register(uint8_t reg) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_MAG_SENSOE_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_MAG_SENSOE_READ, 1);
    return result;
}

uint8_t * LSM9DS1_read_mag_multi(uint8_t reg, uint16_t size) {
    configure_i2c(); 
    // st
    start_i2c_communication();
    // SAD + W
    i2c_send_adress(LSM9DS1_MAG_SENSOE_WRITE);
    // SUB
    i2c_send_data(reg);
    // SR
    start_i2c_communication();
    // SAD + R + rec
    uint8_t * result = i2c_recieve_data(LSM9DS1_MAG_SENSOE_READ, size);
    return result;
}

#define WRITE LSM9DS1_write_acc_and_gyro_register
#define READ LSM9DS1_read_acc_and_gyro_register

// data values
Vec3 LSM9DS1_gyro;
double LSM9DS1_gyro_availiable = 0;

Vec3 LSM9DS1_acc;
double LSM9DS1_acc_availiable = 0;

Vec3 LSM9DS1_mag;
double LSM9DS1_mag_availiable = 0;

Vec3 acc_bias;
Vec3 gyro_bias;
Vec3 mag_bias;

void LSM9DS1_reset() {
    WRITE(CTRL_REG8, 0b0000001);
}

void LSM9DS1_configure_gyro() {
    // general config
    uint8_t odr = (0b110 << 5); // 476 Hz
    uint8_t fs = (0b01 << 3); // 500 dps
    uint8_t bw = (0b00 << 0); // default bw
    uint8_t ctrl_reg1_g = odr | fs | bw;
    WRITE(CTRL_REG1_G, ctrl_reg1_g);
    // enable highpass filter
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b1001<< 0); // Frequenzy 1 
    uint8_t ctrl_reg3_g = hp | hpcf ;
    WRITE(CTRL_REG3_G, ctrl_reg3_g);
}


void LSM9DS1_configure_accel() {
    // general config
    uint8_t odr = (0b100 << 5); // 952 Hz
    uint8_t fs = (0b00 << 3); // 2 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b01 << 0); // default bw, dosnt matter
    uint8_t ctrl_reg6_xl = odr | fs | bw;
    WRITE(CTRL_REG6_XL, ctrl_reg6_xl);
}

void LSM9DS1_configure_mag() {
    uint8_t ctrl_reg1_m = 0b11111100;
    uint8_t ctrl_reg2_m = 0b00000000;
    uint8_t ctrl_reg3_m = 0b00000000;
    uint8_t ctrl_reg4_m = 0b00001100;
    LSM9DS1_write_mag_register(CTRL_REG1_M, ctrl_reg1_m);
    LSM9DS1_write_mag_register(CTRL_REG2_M, ctrl_reg2_m);
    LSM9DS1_write_mag_register(CTRL_REG3_M, ctrl_reg3_m);
    LSM9DS1_write_mag_register(CTRL_REG4_M, ctrl_reg4_m);
}

void LSM9DS1_calibrate_sensors() {
    gyro_bias = gyro_bias * 0;
    acc_bias = acc_bias * 0;
    mag_bias = mag_bias * 0;

    Vec3 temp_gyro;
    Vec3 temp_acc;
    Vec3 temp_mag;

    sleep(1 * SECONDS);
    int gyro_cnt = 0;
    int acc_cnt = 0;
    int mag_cnt = 0;
    
    for (int i = 0 ; i < NBR_CALIB; ++i) {
        LSM9DS1_read_status();
        if (LSM9DS1_gyro_availiable) {
            LSM9DS1_read_gyro();
            temp_gyro = temp_gyro + LSM9DS1_gyro;
            gyro_cnt++;
        }
        if (LSM9DS1_acc_availiable) {
            LSM9DS1_read_accel();
            temp_acc = temp_acc + LSM9DS1_acc;
            acc_cnt++;
        }
        if (LSM9DS1_mag_availiable) {
            LSM9DS1_read_mag();
            temp_mag = temp_mag + LSM9DS1_mag;
            mag_cnt++;
        }
        sleep(3 * MILLISECONDS);
    }
    gyro_bias = temp_gyro / gyro_cnt;
    //acc_bias = temp_acc / acc_cnt;
    //acc_bias[2] += 1;
    // mag
    double mag_x_max = 0.83, mag_x_min = -0.101;
    double mag_y_max = 0.86, mag_y_min = -0.079;
    double mag_z_max = -0.707, mag_z_min = -1.58;
    mag_bias[0] = (mag_x_max + mag_x_min) / 2;
    mag_bias[1] = (mag_x_max + mag_x_min) / 2;
    mag_bias[2] = (mag_z_max + mag_z_min) / 2;
}

void LSM9DS1_read_status() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_register(LSM9DS1_STATUS_REG);
    LSM9DS1_gyro_availiable = (*data & 0b10);
    LSM9DS1_acc_availiable = (*data & 0b1);
    os_free(data);
    data = LSM9DS1_read_mag_register(LSM9DS1_STATUS_REG_M);
    LSM9DS1_mag_availiable = (*data & 0b1000);
    os_free(data);
}

void LSM9DS1_read_gyro() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_multi(OUT_X_G_L, 6);
    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];
    LSM9DS1_gyro[0] = (double) (x * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[1] = (double) (y * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro[2] = (double) (z * GYRO_SENSITIVITY) / 1000 ;
    LSM9DS1_gyro = LSM9DS1_gyro - gyro_bias;
    os_free(data);
}

void LSM9DS1_read_accel() {
    uint8_t * data = LSM9DS1_read_acc_and_gyro_multi(OUT_X_XL_L, 6);
    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];
    LSM9DS1_acc[0] = (double) (x * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[1] = (double) (y * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc[2] = (double) (z * ACC_SENSITIVITY) / 1000 ;
    LSM9DS1_acc = LSM9DS1_acc - acc_bias;
    os_free(data);
}

void LSM9DS1_read_mag() {
    uint8_t * data = LSM9DS1_read_mag_multi(OUT_X_L_M, 6);
    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];
    LSM9DS1_mag[0] = (double) (x * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[1] = (double) (y * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag[2] = (double) (z * MAG_SENSITIVITY) / 1000 ;
    LSM9DS1_mag = LSM9DS1_mag - mag_bias;
    os_free(data);
}

EKF ekf;

Vec3 gyro[100];
Vec3 mag[100];
Vec3 acc[100];
void LSM9DS1_thread() {
  uint8_t tst = now();
  LSM9DS1_reset();
  LSM9DS1_configure_gyro();
  LSM9DS1_configure_accel();
  LSM9DS1_configure_mag();
  // calibration
  LSM9DS1_calibrate_sensors();
  int gyro_cnt = 0;
  int acc_cnt = 0;
  int mag_cnt = 0;
  uint8_t has_init = 0;
  uint64_t last_time = now();
  while (1) {
    LSM9DS1_read_status();
    if (LSM9DS1_gyro_availiable) {
      LSM9DS1_read_gyro();
      //os_printf("LSM9DS1_gyro, ");
      //LSM9DS1_gyro.print_bare();
      gyro_cnt++;
      if (gyro_cnt < 100)
        gyro[gyro_cnt] = LSM9DS1_gyro;
      if (has_init) {
        uint8_t n = now();
        ekf.predict(LSM9DS1_gyro * M_PI / 180, (now() - last_time)/ 100);
        last_time = now();
        ekf.update();
        //os_printf("Attitude, ");
        //ekf.attitude.print_bare();
      }
    }
    if (LSM9DS1_acc_availiable) {
      LSM9DS1_read_accel();
      //os_printf("LSM9DS1_acc, ");
      //LSM9DS1_acc.print_bare();
      acc_cnt++;
      if (acc_cnt < 100) {
        acc[acc_cnt] = LSM9DS1_acc;
      }
      if (has_init) {
         ekf.update_acc(LSM9DS1_acc);
      }
    }
    if (LSM9DS1_mag_availiable) {
      LSM9DS1_read_mag();
      //os_printf("LSM9DS1_mag, ");
      //LSM9DS1_mag.print_bare();
      mag_cnt++;
      if (mag_cnt < 100)
        mag[mag_cnt] = LSM9DS1_mag;
      if (has_init) {
         ekf.update_mag(LSM9DS1_mag);
      }
    }
    if (gyro_cnt > 100 && mag_cnt > 100 && acc_cnt > 100 && !has_init) {
      ekf.init(gyro, acc, mag);
      ekf.update_acc(LSM9DS1_acc);
      ekf.update_mag(LSM9DS1_mag);
      has_init = 1;
    }

    sleep(3 * MILLISECONDS);
  }
}
