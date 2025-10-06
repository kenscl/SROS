#include "LSM9DS1.h"
#include "../communication/SPI.h"
#include "../communication/usart.h"
#include "../krnl/scheduler.h"
#include "../krnl/thread.h"
#include <stdint.h>

Vec gyro_bias;
Mat soft_iron;
Vec hard_iron;
Mat acc_scale;
Vec acc_bias;

/*
 * Calibration values here
 */

void LSM9DS1_calibrate_sensors() {
    //    // Gyroscope
    //    gyro_bias[0] = -0.564394;
    //    gyro_bias[1] = -1.807167;
    //    gyro_bias[2] = -1.857611;
    //  // Magnetometer
    //    soft_iron[0][0] = 0.780890;
    //    soft_iron[0][1] = -0.018096;
    //    soft_iron[0][2] = 0.005514;
    //    soft_iron[1][0] = -0.011893;
    //    soft_iron[1][1] = 0.703616;
    //    soft_iron[1][2] = 0.002129;
    //    soft_iron[2][0] = 0.005069;
    //    soft_iron[2][1] = 0.002974;
    //    soft_iron[2][2] = 0.764143;
    //    hard_iron[0] = 0.226322;
    //    hard_iron[1] = 0.143636;
    //    hard_iron[2] = -0.010043;
    //    // Accelerometer
    //    acc_bias[0] = -0.004453;
    //    acc_bias[1] = 0.002806;
    //    acc_bias[2] = -0.001830;
    //
    //    acc_scale[0][0] = 6.007124;
    //    acc_scale[0][1] = 0.000000;
    //    acc_scale[0][2] = 0.000000;
    //
    //    acc_scale[1][0] = 0.000000;
    //    acc_scale[1][1] = 6.035877;
    //    acc_scale[1][2] = 0.000000;
    //
    //    acc_scale[2][0] = 0.000000;
    //    acc_scale[2][1] = 0.000000;
    //    acc_scale[2][2] = 5.952594;
}

// data values
Vec *LSM9DS1_gyro;
Vec *LSM9DS1_acc;
Vec *LSM9DS1_mag;
Vec *LSM9DS1_gyro_filtered;
float LSM9DS1_gyro_availiable = 0;
Vec *LSM9DS1_acc_filtered;
float LSM9DS1_acc_availiable = 0;
Vec *LSM9DS1_mag_filtered;
float LSM9DS1_mag_availiable = 0;

uint8_t dummy_rx[2] = {};
uint8_t data[2];

// filter constants

float a_acc = 0.15;
float a_gyro = 0.2;
float a_mag = 0.15;

void LSM9DS1_reset() {
    data[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG8);
    data[1] = 0b10000101;
    static struct SPI_transmition info = {.rx_buffer = dummy_rx,
                                          .tx_buffer = data,
                                          .size = 2,
                                          .cs_high = &CS_A_H,
                                          .cs_low = &CS_A_L,
                                          .state = pending};

    while (!SPI_submit(&info));
}

uint8_t data1_g[2], data3_g[2];
void LSM9DS1_configure_gyro() {
    // general config
    uint8_t odr = (0b110 << 5); // 952 Hz
    uint8_t fs = (0b01 << 3);   // 500 dps
    uint8_t bw = (0b00 << 0);   // default bw
    data1_g[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG1_G);
    data1_g[1] = odr | fs | bw;
    static struct SPI_transmition info1_g = {.rx_buffer = dummy_rx,
                                          .tx_buffer = data1_g,
                                          .size = 2,
                                          .cs_high = &CS_A_H,
                                          .cs_low = &CS_A_L,
                                          .state = pending};

    while (!SPI_submit(&info1_g));

    // enable highpass filter
    data3_g[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG3_G);
    uint8_t hp = (0b1 << 6);
    uint8_t hpcf = (0b1001 << 0); // Frequenzy 1
    data3_g[1] = hp | hpcf;

    static struct SPI_transmition info3_g = {.rx_buffer = dummy_rx,
                                          .tx_buffer = data3_g,
                                          .size = 2,
                                          .cs_high = &CS_A_H,
                                          .cs_low = &CS_A_L,
                                          .state = pending};
    while (!SPI_submit(&info3_g));
}

uint8_t data6_xl[2];

void LSM9DS1_configure_accel() {
    // general config
    uint8_t odr = (0b110 << 5);    // 952 Hz
    uint8_t fs = (0b01 << 3);      // 4 g
    uint8_t bw_scale = (0b0 << 2); // bw according to odr
    uint8_t bw = (0b01 << 0);      // default bw, dosnt matter
    data6_xl[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG6_XL);
    data6_xl[1] = odr | fs | bw;

    static struct SPI_transmition info6_xl = {.rx_buffer = dummy_rx,
                                              .tx_buffer = data6_xl,
                                              .size = 2,
                                              .cs_high = &CS_A_H,
                                              .cs_low = &CS_A_L,
                                              .state = pending};
    while (!SPI_submit(&info6_xl));
}

uint8_t data1_m[2], data2_m[2], data3_m[2], data4_m[2];
uint8_t ctrl_reg1_m = 0b11111110;
uint8_t ctrl_reg2_m = 0b00000000;
uint8_t ctrl_reg3_m = 0b00000000; // sim needs to be 0, this is an error in the datasheet!
uint8_t ctrl_reg4_m = 0b00001100;
void LSM9DS1_configure_mag() {
    data1_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG1_M);
    data2_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG2_M);
    data3_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG3_M);
    data4_m[0] = LSM9DS1_WRITE_REGISTER(CTRL_REG4_M);

    data1_m[1] = ctrl_reg1_m;
    data2_m[1] = ctrl_reg2_m;
    data3_m[1] = ctrl_reg3_m;
    data4_m[1] = ctrl_reg4_m;

    static struct SPI_transmition info1_m = {.rx_buffer = dummy_rx,
                                              .tx_buffer = data1_m,
                                              .size = 2,
                                              .cs_high = &CS_M_H,
                                              .cs_low = &CS_M_L,
                                              .state = pending};
    static struct SPI_transmition info2_m = {.rx_buffer = dummy_rx,
                                              .tx_buffer = data2_m,
                                              .size = 2,
                                              .cs_high = &CS_M_H,
                                              .cs_low = &CS_M_L,
                                              .state = pending};
    static struct SPI_transmition info3_m = {.rx_buffer = dummy_rx,
                                              .tx_buffer = data3_m,
                                              .size = 2,
                                              .cs_high = &CS_M_H,
                                              .cs_low = &CS_M_L,
                                              .state = pending};
    static struct SPI_transmition info4_m = {.rx_buffer = dummy_rx,
                                              .tx_buffer = data4_m,
                                              .size = 2,
                                              .cs_high = &CS_M_H,
                                              .cs_low = &CS_M_L,
                                              .state = pending};


    while (!SPI_submit(&info1_m));
    while (!SPI_submit(&info2_m));
    while (!SPI_submit(&info3_m));
    while (!SPI_submit(&info4_m));
}

uint8_t status_reg[2] = {};
uint8_t status_reg_tx[2] = {};
uint8_t status_reg_m[2] = {};
uint8_t status_reg_m_tx[2] = {};

static struct SPI_transmition acc_and_gyro_status = {.rx_buffer = status_reg,
                                         .tx_buffer = status_reg_tx,
                                         .size = 2,
                                         .cs_high = &CS_A_H,
                                         .cs_low = &CS_A_L,
                                         .state = pending};
static struct SPI_transmition mag_status = {.rx_buffer = status_reg_m,
                                            .tx_buffer = status_reg_m_tx,
                                            .size = 2,
                                            .cs_high = &CS_M_H,
                                            .cs_low = &CS_M_L,
                                            .state = pending};

void LSM9DS1_read_status() {
    if (LSM9DS1_check_status())
        return; // return if message is already pending
    status_reg_tx[0] = LSM9DS1_READ_REGISTER(LSM9DS1_STATUS_REG);
    status_reg_m_tx[0] = LSM9DS1_READ_REGISTER(LSM9DS1_STATUS_REG_M);
    SPI_submit(&acc_and_gyro_status);
    SPI_submit(&mag_status);
}

uint8_t LSM9DS1_check_status() {
    if (acc_and_gyro_status.state == done && mag_status.state == done)
        return 1;
    else
        return 0;
}

void LSM9DS1_enable_status() {
    acc_and_gyro_status.state = pending;
    mag_status.state = pending;
}

//#pragma GCC push_options
//#pragma GCC optimize("O0")
void LSM9DS1_process_status() {
    if (!LSM9DS1_check_status())
        return;
    LSM9DS1_gyro_availiable = (status_reg[1] & 0b10);
    LSM9DS1_acc_availiable = (status_reg[1] & 0b1);
    LSM9DS1_mag_availiable = (status_reg_m[1] & 0b1000);
    LSM9DS1_enable_status();
}
//#pragma GCC pop_options

uint8_t gyro_data[7] = {};
uint8_t gyro_data_tx[7] = {};
static struct SPI_transmition gyro_data_reg = {.rx_buffer = gyro_data,
                                            .tx_buffer = gyro_data_tx,
                                            .size = 7,
                                            .cs_high = &CS_A_H,
                                            .cs_low = &CS_A_L,
                                            .state = pending};

void LSM9DS1_read_gyro() {
    if (LSM9DS1_check_gyro())
        return;
    gyro_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_G_L);
    for (int i = 1; i < 7; ++i) {
        gyro_data_tx[i] = 0x00;
    }
    SPI_submit(&gyro_data_reg);
}

uint8_t LSM9DS1_check_gyro() {
    if (gyro_data_reg.state == done)
        return 1;
    else
        return 0;
}

void LSM9DS1_enable_gyro() {
    gyro_data_reg.state = pending;
}

void LSM9DS1_process_gyro() {
    if (!LSM9DS1_check_gyro())
        return;
    volatile int16_t x = (gyro_data[1 + 1] << 8) | gyro_data[0 + 1];
    volatile int16_t y = (gyro_data[3 + 1] << 8) | gyro_data[2 + 1];
    volatile int16_t z = (gyro_data[5 + 1] << 8) | gyro_data[4 + 1];
    LSM9DS1_gyro->r[0] = (float)(x * GYRO_SENSITIVITY) / 1000;
    LSM9DS1_gyro->r[1] = -(float)(y * GYRO_SENSITIVITY) / 1000;
    LSM9DS1_gyro->r[2] = -(float)(z * GYRO_SENSITIVITY) / 1000;
    vec_sub(LSM9DS1_gyro, &gyro_bias, LSM9DS1_gyro);
    LSM9DS1_gyro->r[1] = -LSM9DS1_gyro->r[1];
    //LSM9DS1_gyro_filtered = low_pass_filter(a_gyro, LSM9DS1_gyro_filtered, LSM9DS1_gyro);
    LSM9DS1_enable_gyro();
}

uint8_t acc_data[7] = {};
uint8_t acc_data_tx[7] = {};

static struct SPI_transmition acc_data_reg = {.rx_buffer = acc_data,
                                            .tx_buffer = acc_data_tx,
                                            .size = 7,
                                            .cs_high = &CS_A_H,
                                            .cs_low = &CS_A_L,
                                            .state = pending};

void LSM9DS1_read_accel() {
    if (LSM9DS1_check_accel())
        return;
    acc_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_XL_L);
    for (int i = 1; i < 7; ++i) {
        acc_data_tx[i] = 0x00;
    }
    SPI_submit(&acc_data_reg);
}

uint8_t LSM9DS1_check_accel() {
    if (acc_data_reg.state == done)
        return 1;
    else
        return 0;
}

void LSM9DS1_enable_accel() {
    acc_data_reg.state = pending;
}

void LSM9DS1_process_accel() {
    if (!LSM9DS1_check_accel())
        return;
    int16_t x = (acc_data[1 + 1] << 8) | acc_data[0 + 1];
    int16_t y = (acc_data[3 + 1] << 8) | acc_data[2 + 1];
    int16_t z = (acc_data[5 + 1] << 8) | acc_data[4 + 1];
    LSM9DS1_acc->r[0] = -(float)(x * ACC_SENSITIVITY) / 1000;
    LSM9DS1_acc->r[1] = (float)(y * ACC_SENSITIVITY) / 1000;
    LSM9DS1_acc->r[2] = (float)(z * ACC_SENSITIVITY) / 1000;
    vec_add(LSM9DS1_acc, &acc_bias, LSM9DS1_acc);
    Vec *tmp = vec_alloc(3);
    if (tmp == 0) return;
    mat_vec_mult(&acc_scale, LSM9DS1_acc, tmp);
    LSM9DS1_acc->r[0] = tmp->r[0];
    LSM9DS1_acc->r[1] = tmp->r[1];
    LSM9DS1_acc->r[2] = tmp->r[2];
    vec_free(tmp);
    LSM9DS1_acc->r[1] = -LSM9DS1_acc->r[1];

    float res = 1 - vec_norm(LSM9DS1_acc);
    //res = res * res;
    //if (res < 0.1) {
    //    LSM9DS1_acc_filtered =
    //        low_pass_filter(a_acc, LSM9DS1_acc_filtered, LSM9DS1_acc.normalize());
    //}
    LSM9DS1_enable_accel();
}

uint8_t mag_data[7] = {};
uint8_t mag_data_tx[7] = {};

static struct SPI_transmition mag_data_reg = {.rx_buffer = mag_data,
                                            .tx_buffer = mag_data_tx,
                                            .size = 7,
                                            .cs_high = &CS_M_H,
                                            .cs_low = &CS_M_L,
                                            .state = pending};

void LSM9DS1_read_mag() {
    mag_data_tx[0] = LSM9DS1_READ_REGISTER(OUT_X_L_M);
    for (int i = 1; i < 7; ++i) {
        mag_data_tx[i] = 0x00;
    }
    SPI_submit(&mag_data_reg);
}

uint8_t LSM9DS1_check_mag() {
    if (mag_data_reg.state == done)
        return 1;
    else
        return 0;
}

void LSM9DS1_enable_mag() {
    mag_data_reg.state = pending;
}

void LSM9DS1_process_mag() {
    if (!LSM9DS1_check_mag())
        return;
    int16_t x = (mag_data[1 + 1] << 8) | mag_data[0 + 1];
    int16_t y = (mag_data[3 + 1] << 8) | mag_data[2 + 1];
    int16_t z = (mag_data[5 + 1] << 8) | mag_data[4 + 1];
    LSM9DS1_mag->r[0] = -(float)(x * MAG_SENSITIVITY) / 1000;
    LSM9DS1_mag->r[1] = (float)(y * MAG_SENSITIVITY) / 1000;
    LSM9DS1_mag->r[2] = -(float)(z * MAG_SENSITIVITY) / 1000;
    vec_sub(LSM9DS1_mag, &hard_iron, LSM9DS1_mag);

    Vec *tmp = vec_alloc(3);
    if (tmp == 0) return;
    mat_vec_mult(&soft_iron, LSM9DS1_mag, tmp);
    LSM9DS1_mag->r[0] = tmp->r[0];
    LSM9DS1_mag->r[1] = tmp->r[1];
    LSM9DS1_mag->r[2] = tmp->r[2];
    vec_free(tmp);
    //LSM9DS1_mag_filtered = low_pass_filter(a_mag, LSM9DS1_mag_filtered, LSM9DS1_mag.normalize());
    LSM9DS1_enable_mag();
}

uint8_t who_data[2], who_data_m[2] = {};
uint8_t who_data_tx[2], who_data_tx_m[2] = {};

static struct SPI_transmition who_data_reg = {.rx_buffer = who_data,
                                            .tx_buffer = who_data_tx,
                                            .size = 2,
                                            .cs_high = &CS_A_H,
                                            .cs_low = &CS_A_L,
                                            .state = pending};

static struct SPI_transmition who_data_reg_m = {.rx_buffer = who_data_m,
                                            .tx_buffer = who_data_tx_m,
                                            .size = 2,
                                            .cs_high = &CS_M_H,
                                            .cs_low = &CS_M_L,
                                            .state = pending};

void LSM9DS1_read_WHO_AM_I() {
    if (who_data_reg.state == done)
        return;
    if (who_data_reg_m.state == done)
        return;
    who_data_tx[0] = LSM9DS1_READ_REGISTER(LSM9DS1_WHO_AM_I);
    who_data_tx_m[0] = LSM9DS1_READ_REGISTER(LSM9DS1_WHO_AM_I);
    SPI_submit(&who_data_reg);
    SPI_submit(&who_data_reg_m);
}

uint8_t LSM9DS1_check_WHO_AM_I() {
    if (who_data_reg.state == done && who_data_reg_m.state == done)
        return 1;
    else
        return 0;
}

void LSM9DS1_enable_WHO_AM_I() {
    who_data_reg.state = pending;
    who_data_reg_m.state = pending;
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

//Vec  low_pass_filter(float alpha, Vec mean, Vec new_measurement) {
//    vec_scalar_mult(&mean, alpha);
//    vec_scalar_mult(&new_measurement, 1 - alpha);
//    Vec *ret = vec_alloc()
//    return ret;
//}

uint8_t eq_cnt = 0;
uint32_t last_time = 0;
uint32_t next_mag = 0;
volatile void LSM9DS1_thread() {
    LSM9DS1_gyro = vec_alloc(3);
    LSM9DS1_mag = vec_alloc(3);
    LSM9DS1_acc = vec_alloc(3);
    SPI_init();
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
        volatile uint32_t next_time = now() + 3 * MILLISECONDS;
        last_time = now();

        LSM9DS1_read_gyro();
        if (next_mag < now()) {
            LSM9DS1_read_mag();
            LSM9DS1_read_accel();
            next_mag = now() + 50 * MILLISECONDS;
        }

        SPI_handle();

        sleep(2 * MILLISECONDS);

        Vec *last_gyro = vec_alloc(3);
        last_gyro->r[0] = LSM9DS1_gyro->r[0];
        last_gyro->r[1] = LSM9DS1_gyro->r[1];
        last_gyro->r[2] = LSM9DS1_gyro->r[2];


        LSM9DS1_process_gyro();
        LSM9DS1_process_accel();
        LSM9DS1_process_mag();
        // os_printf("LSM9DS1_mag, ");
        // LSM9DS1_mag.print_bare();

        if (vec_equals(last_gyro, LSM9DS1_gyro)) {
            eq_cnt++;
        } else {
            eq_cnt = 0;
        }
        vec_free(last_gyro);

        sleep_until(next_time);
        if (eq_cnt > 10) {
            eq_cnt = 0;
            os_printf("\n\n\nconnection error! \n\n\n");
            SPI_init();
            sleep(1 * MILLISECONDS);
            LSM9DS1_reset();
            SPI_handle();
            sleep(1 * MILLISECONDS);

            LSM9DS1_configure_gyro();
            LSM9DS1_configure_accel();
            LSM9DS1_configure_mag();
            LSM9DS1_read_WHO_AM_I();
            SPI_handle();
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
