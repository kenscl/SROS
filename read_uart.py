import serial
import time

port = '/dev/ttyUSB0'  
baudrate = 115200 

ser = serial.Serial(port, baudrate)

output_file = open('serial_data.txt', 'w')
count = 0
last_time = 0
while True:
    try:
        try:
            while True:
                data = ser.readline().decode('utf-8').strip()  
                print(data)
                count += 1
                if (last_time + 1< time.time()):
                    print("nbr of messages per second: %d" % count)
                    last_time = time.time()
                    count = 0

                
                
        except KeyboardInterrupt:
            ser.close()
            output_file.close()
    except UnicodeDecodeError:
        print("err")
        pass
