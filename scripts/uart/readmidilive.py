import serial
import datetime

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
print(ser)
ser.open()

last_midi = 0

while (True):
    new_data = ser.read(3)
    if (new_data[1] != last_midi):
        last_midi = new_data[1]
    print(last_midi)

