import serial
import datetime

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
print(ser)
ser.open()


data1 = ser.read(int(1250*4))
#data2 = ser.read(int(44100))
uniq_filename = str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '.')
with open("bufferdata/" + uniq_filename + ".dat", 'wb') as f:
    f.write(data1)
    #f.write(data2)
print("Read buffer")

