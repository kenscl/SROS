import serial
import numpy as np
from scipy import linalg
import asyncio

calib_gyro = False
calib_acc = False
calib_mag = True

port = '/dev/ttyUSB0'
baudrate = 115200

import serial
import numpy as np
import matplotlib.pyplot as plt
import time


def ellipsoid_fit(data):
    data = np.array(data)

    # since traditioinal Least squares tends to return x = 0 if A*x = 0 well use svd instead and solve for the null space of A
    x, y, z = data[:, 0], data[:, 1], data[:, 2]

    A = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z, np.ones_like(x)))

    # Solve for the parameters
    _, _, Vt = np.linalg.svd(A, full_matrices=False)
    return Vt[-1, :]


class Calib:
    def __init__(self):
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200

        self.ser = serial.Serial(self.port, self.baudrate)

        self.last_acc = list()
        self.last_gyro = np.array([0., 0., 0.])
        self.last_mag = np.array([0., 0., 0.])
        self.last_attitude = np.array([0., 0., 0., 0.,])
        self.acc = list()
        self.gyro = list()
        self.mag = list()
        self.attitude = list()
        self.time_stamp_gyro = list()
        self.time_stamp_acc = list()
        self.time_stamp_mag = list()
        self.time_stamp_attitude = list()
        self.start = time.time()

        self.acc_x_data, self.acc_y_data, self.acc_z_data = [], [], []
        self.gyro_x_data, self.gyro_y_data, self.gyro_z_data = [], [], []
        self.mag_x_data, self.mag_y_data, self.mag_z_data = [], [], []

        self.plot_gyro = True
        self.plot_acc = False
        self.plot_mag = False
        self.plot_attitude = True
        self.last_n = list()
        self.ser = serial.Serial(self.port, self.baudrate)

    def gather_data(self, count):
        self.ser.flushInput()
        try:
            try:
                while (count > 0):
                    data = self.ser.readline().decode('utf-8').strip().replace("\x00", "")
                    self.last_n.append(data)
                    now = time.time() - self.start
                    try:
                        sensor, _VEC, _, x, y, z, _ = data.split(" ")
                        if str(sensor) == str('[LSM9DS1_acc]'):
                            last_acc = np.array([float(x), float(y), float(z)])
                            self.acc_x_data.append(last_acc[0])
                            self.acc_y_data.append(last_acc[1])
                            self.acc_z_data.append(last_acc[2])
                            self.acc.append(last_acc)
                            self.time_stamp_acc.append(now)

                        elif str(sensor) == str('[LSM9DS1_gyro]'):
                            last_gyro = np.array([float(x), float(y), float(z)])
                            self.gyro_x_data.append(last_gyro[0])
                            self.gyro_y_data.append(last_gyro[1])
                            self.gyro_z_data.append(last_gyro[2])
                            self.gyro.append(last_gyro)
                            self.time_stamp_gyro.append(now)

                        elif str(sensor) == str('[LSM9DS1_mag]'):
                            last_mag = np.array([float(x), float(y), float(z)])
                            self.mag_x_data.append(last_mag[0])
                            self.mag_y_data.append(last_mag[1])
                            self.mag_z_data.append(last_mag[2])
                            self.mag.append(last_mag)
                            self.time_stamp_mag.append(now)
                        count -= 1
                        print("Steps remaining: %d" % count)
                    except Exception as e:
                        pass
            except KeyboardInterrupt:
                pass
        except Exception as e:
            print("Error: ", e)
            pass

    def gyro_calib(self) -> np.ndarray:
        wait = True
        while (wait):
            res = input("Gyroscope Calibration: \nKeep the sensor Steady after countdown. \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Gyroscope calibration in 3...")
        time.sleep(1)
        print("Starting Gyroscope calibration in 2...")
        time.sleep(1)
        print("Starting Gyroscope calibration in 1...")
        time.sleep(1)
        self.gyro = list()
        self.gather_data(1000)
        return np.mean(self.gyro, axis=0)

    def magnetometer_calib(self):
        self.mag = list()
        wait = True
        while (wait):
            res = input("Magnetometer Calibration: \n. Rotate the Magneteometer about the Positive X-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Magnetometer calibration in 3...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 2...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 1...")
        time.sleep(1)
        self.gather_data(2000)

        wait = True
        while (wait):
            res = input("Magnetometer Calibration: \n. Rotate the Magneteometer about the Positive Y-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Magnetometer calibration in 3...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 2...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 1...")
        time.sleep(1)
        self.gather_data(2000)

        wait = True
        while (wait):
            res = input("Magnetometer Calibration: \n. Rotate the Magneteometer about the Positive Z-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Magnetometer calibration in 3...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 2...")
        time.sleep(1)
        print("Starting Magnetometer calibration in 1...")
        time.sleep(1)
        self.gather_data(2000)

        # see https://www.ensta-bretagne.fr/lebars/Share/RI_magnetometer.pdf for ellipseoid stuff
        #  the solution is my own since i didnt understand theirs
        params = ellipsoid_fit(self.mag)
        A, B, C, D, E, F, G, H, I, J = params

        offset = np.array([-G/A, -H/B, -I/C])

        M_inv = np.array([
            [1/A, D/A, E/A],
            [D/B, 1/B, F/B],
            [E/C, F/C, 1/C]
        ])

        U, S, Vt = np.linalg.svd(M_inv)
        soft_iron_matrix = np.dot(U, np.dot(np.diag(1/np.sqrt(S)), Vt))
        return soft_iron_matrix, offset

    def accelerometer_calib(self):
        calib_count = 10
        self.acc = list()
        wait = True
        print("Move the Accelerometer to the requestet position and move it around slightly.")
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer the Positive Z-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        zp = self.acc
        self.acc = list()

        wait = True
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer to the Negative Z-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        zm = self.acc
        self.acc = list()

        wait = True
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer to the Positive X-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        xp = self.acc
        self.acc = list()

        wait = True
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer to the Negative X-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        xm = self.acc
        self.acc = list()

        wait = True
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer to the Positive Y-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        yp = self.acc
        self.acc = list()

        wait = True
        while (wait):
            res = input("Accelerometer Calibration: \n. Move the Accelerometer to the Negative Y-Axis \nProceed? Y/n? ")
            if res == "Y" or res == "y" or res == "yes":
                wait = False
        print("Starting Accelerometer calibration in 3...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 2...")
        time.sleep(1)
        print("Starting Accelerometer calibration in 1...")
        time.sleep(1)
        self.gather_data(calib_count)
        ym = self.acc
        self.acc = list()

        xup = np.max(np.array(xp)[:, 0])
        xdown = np.min(np.array(xm)[:, 0])
        yup = np.max(np.array(yp)[:, 1])
        ydown = np.min(np.array(ym)[:, 1])
        zup = np.min(np.array(zp)[:, 2])
        zdown = np.max(np.array(zm)[:, 2])

        bias = np.array([xup + xdown, yup + ydown, zup + zdown]) / 2
        scale = np.array([[2 / np.abs(xup - xdown), 0, 0], [0,  2 / np.abs(yup - ydown), 0], [0, 0, 2 / np.abs(zup - zdown)]])
        return bias, scale

    def main(self):
        if calib_gyro:
            gyro_bias = self.gyro_calib()
        if calib_mag:
            soft_iron, hard_iron = self.magnetometer_calib()
        if calib_acc:
            acc_bias, acc_scale = self.accelerometer_calib()

        # calibration prints

        print("\n Calibration values: (Input in src/communication/LSM9DS1.cpp line 12)")
        print("void LSM9DS1_calibrate_sensors() {")
        if calib_gyro:
            print("    // Gyroscope")
            print("    gyro_bias.r[0] = %f;" % gyro_bias[0])
            print("    gyro_bias.r[1] = %f;" % gyro_bias[1])
            print("    gyro_bias.r[2] = %f;" % gyro_bias[2])
            print("")
        if calib_mag:
            print("    // Magnetometer")
            print("")
            print("    soft_iron.r[0 * 3 + 0] = %f;" % soft_iron[0][0])
            print("    soft_iron.r[0 * 3 + 1] = %f;" % soft_iron[0][1])
            print("    soft_iron.r[0 * 3 + 2] = %f;" % soft_iron[0][2])
            print("")
            print("    soft_iron.r[1 * 3 + 0] = %f;" % soft_iron[1][0])
            print("    soft_iron.r[1 * 3 + 1] = %f;" % soft_iron[1][1])
            print("    soft_iron.r[1 * 3 + 2] = %f;" % soft_iron[1][2])
            print("")
            print("    soft_iron.r[2 * 3 + 0] = %f;" % soft_iron[2][0])
            print("    soft_iron.r[2 * 3 + 1] = %f;" % soft_iron[2][1])
            print("    soft_iron.r[2 * 3 + 2] = %f;" % soft_iron[2][2])
            print("")
            print("")
            print("    hard_iron.r[0] = %f;" % hard_iron[0])
            print("    hard_iron.r[1] = %f;" % hard_iron[1])
            print("    hard_iron.r[2] = %f;" % hard_iron[2])
            print("")
        if calib_acc:
            print("    // Accelerometer")
            print("")
            print("    acc_bias.r[0] = %f;" % acc_bias[0])
            print("    acc_bias.r[1] = %f;" % acc_bias[1])
            print("    acc_bias.r[2] = %f;" % acc_bias[2])
            print("")
            print("    acc_scale.r[0 * 3 + 0] = %f;" % acc_scale[0][0])
            print("    acc_scale.r[0 * 3 + 1] = %f;" % acc_scale[0][1])
            print("    acc_scale.r[0 * 3 + 2] = %f;" % acc_scale[0][2])
            print("")
            print("    acc_scale.r[1 * 3 + 0] = %f;" % acc_scale[1][0])
            print("    acc_scale.r[1 * 3 + 1] = %f;" % acc_scale[1][1])
            print("    acc_scale.r[1 * 3 + 2] = %f;" % acc_scale[1][2])
            print("")
            print("    acc_scale.r[2 * 3 + 0] = %f;" % acc_scale[2][0])
            print("    acc_scale.r[2 * 3 + 1] = %f;" % acc_scale[2][1])
            print("    acc_scale.r[2 * 3 + 2] = %f;" % acc_scale[2][2])
        print("}")




calib = Calib()
calib.main()
