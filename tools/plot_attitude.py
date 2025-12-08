import serial
import numpy as np
import matplotlib.pyplot as plt
import time
from math_helper import quat2euler
import multiprocessing


port = '/dev/ttyUSB0'
baudrate = 115200

ser = serial.Serial(port, baudrate)


att_rpy = list()
last_n = list()

start = time.time()
def read_data(queue, shutdownEvent):
    try:
        try:
            while (True):
                data = ser.readline().decode('utf-8').strip().replace("\x00", "")
                last_n.append(data)
                now = time.time() - start
                try:
                    print(data)
                    sensor, _Vector, _bracket1, _q, _i, _j, _k, _bracket2 = data.split(" ")
                    if str(sensor) == str('Attitude:'):
                        last_attitude = np.array([float(_q), float(_i), float(_j), float(_k)])

                        attitude = [now, float(_q), float(_i), float(_j), float(_k)]

                        yaw, pitch, roll = quat2euler(last_attitude)
                        att_rpy.append(np.array([roll, pitch, yaw]) * 180 / np.pi)
                        queue.put(attitude)

                except ValueError:
                    pass

                #print("\033c", end="") # clear terminal
                #print("Time: ", now)
                #print("Attitude: ", last_attitude)
                if shutdownEvent.is_set():
                    break

        except KeyboardInterrupt:
            pass
        ser.close()

    except UnicodeDecodeError:
        print("Error decoding data.")
        pass


queue = multiprocessing.Queue(maxsize=100)
shutdownEvent = multiprocessing.Event()
serialProcess = multiprocessing.Process(target=read_data, args=(queue, shutdownEvent))
serialProcess.start()

fig = plt.figure("Attitude")
subplot1 = fig.add_subplot(2, 1, 1)
subplot2 = fig.add_subplot(2, 1, 2)
fig.subplots_adjust(hspace=.5)
t = []
q = []
i = []
j = []
k = []

r = []
p = []
y = []

while True:
    while not queue.empty():
        s = queue.get()
        t.append(s[0])
        q.append(s[1])
        i.append(s[2])
        j.append(s[3])
        k.append(s[4])

        yaw, pitch, roll = quat2euler(np.array([s[1], s[2], s[3], s[4]]))

        r.append(roll * 180 / np.pi)
        p.append(pitch * 180 / np.pi)
        y.append(yaw * 180 / np.pi)

    t = t[-500:]
    q = q[-500:]
    i = i[-500:]
    j = j[-500:]
    k = k[-500:]
    r = r[-500:]
    p = p[-500:]
    y = y[-500:]

    subplot1.clear()
    subplot1.plot(t, q, 'r', t, i, 'g', t, j, 'b', t, k, 'c', linewidth=0.7)
    subplot1.set(xlabel="Time in s", ylabel="Quaternion")
    subplot1.set(ylim=[-1, 1])
    subplot1.legend(["q", "i", "j", "k"], loc='upper right')

    subplot2.clear()
    subplot2.plot(t, r, 'r', t, p, 'g', t, y, 'b', linewidth=0.7)
    subplot2.set(xlabel="Time in s", ylabel="Attitude ")
    subplot2.set(ylim=[-180, 180])
    subplot2.legend(["roll", "pitch", "yaw"], loc='upper right')

    plt.pause(0.001)

shutdownEvent.set()
serialProcess.join()
