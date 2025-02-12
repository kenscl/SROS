import serial

port = '/dev/ttyUSB0'  
baudrate = 115200 

ser = serial.Serial(port, baudrate)

output_file = open('serial_data.txt', 'w')
while True:
    try:
        try:
            while True:
                data = ser.readline().decode('utf-8').strip()  
                
                #output_file.write(data + '\n')  
                
                print(data)
                
        except KeyboardInterrupt:
            ser.close()
            output_file.close()
    except UnicodeDecodeError:
        print("err")
        pass
