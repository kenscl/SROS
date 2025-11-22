#include "LSM9DS1.h"
#include "../communication/SPI.h"
#include "../communication/usart.h"
#include "../globals.h"
#include "../krnl/scheduler.h"
#include "../krnl/thread.h"
#include <stdint.h>

// data values
VEC_ALLOC_STATIC(LSM9DS1_gyro, 3);
VEC_ALLOC_STATIC(LSM9DS1_mag, 3);
VEC_ALLOC_STATIC(LSM9DS1_acc, 3);

VEC_ALLOC_STATIC(LSM9DS1_gyro_filtered, 3);
VEC_ALLOC_STATIC(LSM9DS1_acc_filtered, 3);
VEC_ALLOC_STATIC(LSM9DS1_mag_filtered, 3);

VEC_ALLOC_STATIC(gyro_bias, 3);
VEC_ALLOC_STATIC(hard_iron, 3);
VEC_ALLOC_STATIC(acc_bias, 3);

MAT_ALLOC_STATIC(soft_iron, 3, 3);
MAT_ALLOC_STATIC(acc_scale, 3, 3);

/*
 * Calibration values here
 */

void LSM9DS1_calibrate_sensors() {
    // Gyroscope
    gyro_bias.r[0] = -0.525990;
    gyro_bias.r[1] = 2.009206;
    gyro_bias.r[2] = 1.901428;

    // Magnetometer

    soft_iron.r[0 * 3 + 0] = 0.863373;
    soft_iron.r[0 * 3 + 1] = 0.043898;
    soft_iron.r[0 * 3 + 2] = 0.008326;

    soft_iron.r[1 * 3 + 0] = 0.000861;
    soft_iron.r[1 * 3 + 1] = 0.486198;
    soft_iron.r[1 * 3 + 2] = -0.003300;

    soft_iron.r[2 * 3 + 0] = 0.005396;
    soft_iron.r[2 * 3 + 1] = -0.032683;
    soft_iron.r[2 * 3 + 2] = 0.768943;

    hard_iron.r[0] = 0.164978;
    hard_iron.r[1] = -0.395368;
    hard_iron.r[2] = -0.046471;

    // Accelerometer

    acc_bias.r[0] = -0.008113;
    acc_bias.r[1] = -0.001464;
    acc_bias.r[2] = 0.002196;

    acc_scale.r[0 * 3 + 0] = 6.064908;
    acc_scale.r[0 * 3 + 1] = 0.000000;
    acc_scale.r[0 * 3 + 2] = 0.000000;

    acc_scale.r[1 * 3 + 0] = 0.000000;
    acc_scale.r[1 * 3 + 1] = 5.996138;
    acc_scale.r[1 * 3 + 2] = 0.000000;

    acc_scale.r[2 * 3 + 0] = 0.000000;
    acc_scale.r[2 * 3 + 1] = 0.000000;
    acc_scale.r[2 * 3 + 2] = 5.991755;
}

float LSM9DS1_gyro_availiable = 0;
float LSM9DS1_acc_availiable = 0;
float LSM9DS1_mag_availiable = 0;

uint8_t dummy_rx[2] = {};
uint8_t data[2];

// filter constants

float a_acc = 0.2;
float a_gyro = 0.05;
float a_mag = 0.2;

void LSM9DS1_reset(SPI_INFO *fill) {
    data[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG8);
    data[1] = 0b10000101;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 2;
}

uint8_t data1_g[2], data3_g[2];
void LSM9DS1_WRITE_CTRL_REG1_G(SPI_INFO *fill) {
    uint8_t odr = (0b110 << 5); // 952 Hz
    uint8_t fs = (0b01 << 3);   // 500 dps
    uint8_t bw = (0b00 << 0);   // default bw
    data1_g[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG1_G);
    data1_g[1] = odr | fs | bw;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data1_g;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 2;
}

void LSM9DS1_WRITE_CTRL_REG3_G(SPI_INFO *fill) {
    data3_g[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG3_G);
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b1001 << 0); // Frequenzy 1
    data3_g[1] = hp | hpcf;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data3_g;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 2;
}

uint8_t data6_xl[2];

void LSM9DS1_WRITE_CTRL_REG6_XL(SPI_INFO *fill) {
    // general config
    uint8_t odr = (0b110 << 5);    // 952 Hz
    uint8_t fs = (0b01 << 3);      // 4 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b01 << 0);      // default bw, dosnt matter
    data6_xl[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG6_XL);
    data6_xl[1] = odr | fs | bw;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data6_xl;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 2;
}

uint8_t data1_m[2], data2_m[2], data3_m[2], data4_m[2];
uint8_t ctrl_reg1_m = 0b11111110;
uint8_t ctrl_reg2_m = 0b00000000;
uint8_t ctrl_reg3_m = 0x00; // sim needs to be 0, this is an error in the datasheet!
uint8_t ctrl_reg4_m = 0b00001100;

void LSM9DS1_WRITE_CTRL_REG1_M(SPI_INFO *fill) {
    data1_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG1_M);
    data1_m[1] = ctrl_reg1_m;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data1_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}

void LSM9DS1_WRITE_CTRL_REG2_M(SPI_INFO *fill) {
    data2_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG2_M);
    data2_m[1] = ctrl_reg2_m;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data2_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}

void LSM9DS1_WRITE_CTRL_REG3_M(SPI_INFO *fill) {
    data3_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG3_M);
    data3_m[1] = ctrl_reg3_m;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data3_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}

void LSM9DS1_WRITE_CTRL_REG4_M(SPI_INFO *fill) {
    data4_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG4_M);
    data4_m[1] = ctrl_reg4_m;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = data4_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}

uint8_t who_data[2], who_data_m[2] = {};
uint8_t who_data_tx[2], who_data_tx_m[2] = {};

void LSM9DS1_READ_CTRL_REG4_M(SPI_INFO *fill) {
    who_data_tx_m[0] = LSM9DS1_READ_REGISTER(CTRL_REG4_M);
    who_data_tx_m[1] = 0;
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = dummy_rx;
    fill->tx = who_data_tx_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}


void LSM9DS1_read_WHO_AM_I_A(SPI_INFO *fill) {
    who_data_tx[0] = LSM9DS1_READ_REGISTER(LSM9DS1_WHO_AM_I);
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = who_data;
    fill->tx = who_data_tx;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 2;
}

void LSM9DS1_read_WHO_AM_I_M(SPI_INFO *fill) {
    who_data_tx_m[0] = LSM9DS1_READ_REGISTER(LSM9DS1_WHO_AM_I);
    dummy_rx[0] = 0;
    dummy_rx[1] = 0;

    fill->rx = who_data_m;
    fill->tx = who_data_tx_m;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 2;
}

uint8_t gyro_data[7] = {};
uint8_t gyro_data_tx[7] = {};

void LSM9DS1_read_gyro(SPI_INFO *fill) {
    gyro_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_G_L);
    gyro_data[0] = 0x00;
    for (int i = 1; i < 7; ++i) {
        gyro_data_tx[i] = 0x00;
        gyro_data[i] = 0x00;
    }

    fill->rx = gyro_data;
    fill->tx = gyro_data_tx;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 7;
}

uint8_t acc_data[7] = {};
uint8_t acc_data_tx[7] = {};

void LSM9DS1_read_acc(SPI_INFO *fill) {
    acc_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_XL_L);
    acc_data[0] = 0x00;
    for (int i = 1; i < 7; ++i) {
        acc_data_tx[i] = 0x00;
        acc_data[i] = 0x00;
    }

    fill->rx = acc_data;
    fill->tx = acc_data_tx;
    fill->cs_high = &CS_A_H;
    fill->cs_low = &CS_A_L;
    fill->size = 7;
}

uint8_t mag_data[7] = {};
uint8_t mag_data_tx[7] = {};

void LSM9DS1_read_mag(SPI_INFO *fill) {
    mag_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_L_M);
    mag_data_tx[0] |= (1 << 6);
    mag_data[0] = 0x00;
    for (int i = 1; i < 7; ++i) {
        mag_data_tx[i] = 0x00;
        mag_data[i] = 0x00;
    }

    fill->rx = mag_data;
    fill->tx = mag_data_tx;
    fill->cs_high = &CS_M_H;
    fill->cs_low = &CS_M_L;
    fill->size = 7;
}


void LSM9DS1_process_gyro() {
    volatile int16_t x = (gyro_data[1 + 1] << 8) | gyro_data[0 + 1];
    volatile int16_t y = (gyro_data[3 + 1] << 8) | gyro_data[2 + 1];
    volatile int16_t z = (gyro_data[5 + 1] << 8) | gyro_data[4 + 1];
    LSM9DS1_gyro.r[0] = (float)(x * GYRO_SENSITIVITY) / 1000 * M_PI / 180;
    LSM9DS1_gyro.r[1] = (float)(y * GYRO_SENSITIVITY) / 1000 * M_PI / 180;
    LSM9DS1_gyro.r[2] = (float)(z * GYRO_SENSITIVITY) / 1000 * M_PI / 180;
    vec_sub(&LSM9DS1_gyro, &gyro_bias, &LSM9DS1_gyro);
    low_pass_filter(a_gyro, &LSM9DS1_gyro_filtered, &LSM9DS1_gyro);
}

VEC_ALLOC_STATIC(tmp, 3);
void LSM9DS1_process_accel() {
    int16_t x = (acc_data[1 + 1] << 8) | acc_data[0 + 1];
    int16_t y = (acc_data[3 + 1] << 8) | acc_data[2 + 1];
    int16_t z = (acc_data[5 + 1] << 8) | acc_data[4 + 1];
    LSM9DS1_acc.r[0] = (float)(x * ACC_SENSITIVITY) / 1000;
    LSM9DS1_acc.r[1] = (float)(y * ACC_SENSITIVITY) / 1000;
    LSM9DS1_acc.r[2] = (float)(z * ACC_SENSITIVITY) / 1000;
    vec_add(&LSM9DS1_acc, &acc_bias, &LSM9DS1_acc);
    mat_vec_mult(&acc_scale, &LSM9DS1_acc, &tmp);
    LSM9DS1_acc.r[0] = tmp.r[0];
    LSM9DS1_acc.r[1] = tmp.r[1];
    LSM9DS1_acc.r[2] = tmp.r[2];
    // LSM9DS1_acc.r[1] = -LSM9DS1_acc.r[1];

    // float res = 1 - vec_norm(LSM9DS1_acc);
    //  res = res * res;
    //  if (res < 0.1) {
    low_pass_filter(a_acc, &LSM9DS1_acc_filtered, &LSM9DS1_acc);
    vec_normalize(&LSM9DS1_acc_filtered);
    //  }
}

void LSM9DS1_process_mag() {
    int16_t x = (mag_data[1 + 1] << 8) | mag_data[0 + 1];
    int16_t y = (mag_data[3 + 1] << 8) | mag_data[2 + 1];
    int16_t z = (mag_data[5 + 1] << 8) | mag_data[4 + 1];
    LSM9DS1_mag.r[0] = (float)(y * MAG_SENSITIVITY) / 1000;
    LSM9DS1_mag.r[1] = -(float)(x * MAG_SENSITIVITY) / 1000;
    LSM9DS1_mag.r[2] = (float)(z * MAG_SENSITIVITY) / 1000;
    vec_sub(&LSM9DS1_mag, &hard_iron, &LSM9DS1_mag);

    mat_vec_mult(&soft_iron, &LSM9DS1_mag, &tmp);
    LSM9DS1_mag.r[0] = tmp.r[0];
    LSM9DS1_mag.r[1] = tmp.r[1];
    LSM9DS1_mag.r[2] = tmp.r[2];
    low_pass_filter(a_mag, &LSM9DS1_mag_filtered, &LSM9DS1_mag);
    vec_normalize(&LSM9DS1_mag_filtered);
    // vec_normalize(LSM9DS1_mag));
}



void LSM9DS1_process_WHO_AM_I() {
    if (who_data[1] == 104) {
    } else {
        os_printf("SPI error gyro! \n");
    }
    if (who_data_m[1] == 61) {
    } else {
        os_printf("SPI error mag! %d \n", who_data_m[1]);
    }
}

void low_pass_filter(float alpha, Vec *mean, Vec *new_measurement) {
    vec_scalar_mult(mean, alpha);
    vec_scalar_mult(new_measurement, 1 - alpha);
    vec_add(mean, new_measurement, mean);
}

void process_sensors() {
    LSM9DS1_process_gyro();
    LSM9DS1_process_accel();
    LSM9DS1_process_mag();
}

uint8_t eq_cnt = 0;
uint32_t last_time = 0;
uint32_t next_mag = 0;
VEC_ALLOC_STATIC(comparison, 3);

volatile void LSM9DS1_thread() {
    while (1) {
        volatile uint32_t next_time = now() + 3 * MILLISECONDS;
        sleep(2 * MILLISECONDS);

        process_sensors();

        if (DEBUG == 2) {
            os_printf("[LSM9DS1_gyro] ");
            vec_print(&LSM9DS1_gyro_filtered);

            os_printf("[LSM9DS1_acc] ");
            vec_print(&LSM9DS1_acc_filtered);

            os_printf("[LSM9DS1_mag] ");
            vec_print(&LSM9DS1_mag_filtered);
        }
        sleep_until(next_time);
    }
}
