import serial
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial.transform import Rotation
from math_helper import quat2euler

port = '/dev/ttyUSB0'
baudrate = 115200

ser = serial.Serial(port, baudrate)

last_acc = list()
last_gyro = np.array([0., 0., 0.])
last_mag = np.array([0., 0., 0.])
last_attitude = np.array([0., 0., 0., 0.,])
acc = list()
gyro = list()
mag = list()
attitude = list()
time_stamp_gyro = list()
time_stamp_acc = list()
time_stamp_mag = list()
time_stamp_attitude = list()
start = time.time()

acc_x_data, acc_y_data, acc_z_data = [], [], []
gyro_x_data, gyro_y_data, gyro_z_data = [], [], []
mag_x_data, mag_y_data, mag_z_data = [], [], []
q_att, i_att, j_att, k_att = [], [], [], []
att_rpy = list()

plot_gyro = False
plot_acc = False
plot_mag = True
plot_attitude = False
last_n = list()

try:
    try:
        while (True):
            data = ser.readline().decode('utf-8').strip().replace("\x00", "")
            last_n.append(data)
            now = time.time() - start
            try:
                sensor, _Vector, _bracket1, _q, _i, _j, _k, _bracket2 = data.split(" ")
                if str(sensor) == str('Attitude'):
                    time_stamp_attitude.append(now)
                    last_attitude = np.array([float(_q), float(_i), float(_j), float(_k)])
                    q_att.append(last_attitude[0])
                    i_att.append(last_attitude[1])
                    j_att.append(last_attitude[2])
                    k_att.append(last_attitude[3])

                    attitude.append(last_attitude)

                    yaw, pitch, roll = quat2euler(last_attitude)
                    att_rpy.append(np.array([roll, pitch, yaw]) * 180 / np.pi)
            except ValueError:
                pass
            try:
                sensor, _Vector, _bracket1, x, y, z, _bracket2 = data.split(" ")
                if str(sensor) == str('[LSM9DS1_acc]'):
                    last_acc = np.array([float(x), float(y), float(z)])
                    acc_x_data.append(last_acc[0])
                    acc_y_data.append(last_acc[1])
                    acc_z_data.append(last_acc[2])
                    acc.append(last_acc)
                    time_stamp_acc.append(now)

                elif str(sensor) == str('[LSM9DS1_gyro]'):
                    last_gyro = np.array([float(x), float(y), float(z)]) * np.pi / 180
                    gyro_x_data.append(last_gyro[0])
                    gyro_y_data.append(last_gyro[1])
                    gyro_z_data.append(last_gyro[2])
                    gyro.append(last_gyro)
                    time_stamp_gyro.append(now)

                elif str(sensor) == str('[LSM9DS1_mag]'):
                    last_mag = np.array([float(x), float(y), float(z)])
                    mag_x_data.append(last_mag[0])
                    mag_y_data.append(last_mag[1])
                    mag_z_data.append(last_mag[2])
                    mag.append(last_mag)
                    time_stamp_mag.append(now)

            except ValueError:
                pass
            print("\033c", end="") # clear terminal
            print("Time: ", now)
            print("Accelerometer: ", last_acc)
            print("Gyroscope: ", last_gyro)
            print("Magnetometer: ", last_mag)
            print("Attitude: ", last_attitude)

            for i in reversed(last_n):
                print(i)
            if (len(last_n) > 20):
                last_n = last_n[1:21]
    except KeyboardInterrupt:
        pass
    ser.close()

    plt.figure(figsize=(12, 8))
    if (plot_acc):
        plt.plot(time_stamp_acc, acc_x_data, label="Accelerometer X", color='r')
        plt.plot(time_stamp_acc, acc_y_data, label="Accelerometer Y", color='g')
        plt.plot(time_stamp_acc, acc_z_data, label="Accelerometer Z", color='b')
    if (plot_gyro):
        plt.plot(time_stamp_gyro, gyro_x_data, label="Gyroscope X", color='c')
        plt.plot(time_stamp_gyro, gyro_y_data, label="Gyroscope Y", color='m')
        plt.plot(time_stamp_gyro, gyro_z_data, label="Gyroscope Z", color='y')
        time_stamp_gyro_cp = time_stamp_gyro
        time_stamp_gyro_cp.append(0)
        dt = np.diff(np.array(time_stamp_gyro_cp))
        gyro_x_angle = np.concatenate(([0], np.cumsum(gyro_x_data * dt)))
        gyro_y_angle = np.concatenate(([0], np.cumsum(gyro_y_data * dt)))
        gyro_z_angle = np.concatenate(([0], np.cumsum(gyro_z_data * dt)))

        plt.plot(time_stamp_gyro, gyro_x_angle, label="Integrated Gyro X", color='c')
        plt.plot(time_stamp_gyro, gyro_y_angle, label="Integrated Gyro Y", color='m')
        plt.plot(time_stamp_gyro, gyro_z_angle, label="Integrated Gyro Z", color='y')
    if (plot_mag):
        plt.plot(time_stamp_mag, mag_x_data, label="Magnetometer X", color='k')
        plt.plot(time_stamp_mag, mag_y_data, label="Magnetometer Y", color='orange')
        plt.plot(time_stamp_mag, mag_z_data, label="Magnetometer Z", color='purple')
        #plt.scatter(mag_x_data, mag_y_data, label="XY",  color='orange')
        #plt.scatter(mag_x_data, mag_z_data, label="XZ", color='blue')
        #plt.scatter(mag_y_data, mag_z_data, label="YZ", color='yellow')
    if (plot_attitude):
        plt.plot(time_stamp_attitude, np.array(q_att), label="Attitude q", color='k')
        #plt.plot(time_stamp_attitude, np.array(i_att), label="Attitude i", color='r')
        #plt.plot(time_stamp_attitude, np.array(j_att), label="Attitude j", color='g')
        #plt.plot(time_stamp_attitude, np.array(k_att), label="Attitude k", color='b')
        plt.plot(time_stamp_attitude, np.array(att_rpy)[0:, 0], label="roll")
        plt.plot(time_stamp_attitude, np.array(att_rpy)[0:, 1], label="pitch")
        plt.plot(time_stamp_attitude, np.array(att_rpy)[0:, 2], label="yaw")

    plt.title('Sensor Data (Accelerometer, Gyroscope, Magnetometer)')
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor Values')

    plt.legend()

    plt.show()
    plt.cla()
    plt.clf()

except UnicodeDecodeError:
    print("Error decoding data.")
    pass
