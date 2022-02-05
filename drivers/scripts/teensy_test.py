import serial

def main():
    ser = serial.Serial('/dev/ttyACM0')
    ser.flush()
    print('Flushing')
    while (True):
        line = ser.readline()
        if line != b'this is working\n' and line != b'\r\n':
            print(line)
        # ser.write(b'a')

if __name__ == '__main__':
    main()