import serial
import datetime

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
print(ser)
ser.open()


data = []

for i in range(1):
    new_data = ser.read(int(1250*4))

    print(new_data)
    floats = []
    for i in range(0, len(data), 4):
        floats.append(struct.unpack(">f", data[i:i+4]))
    data.append(floats)



uniq_filename = str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '.')
with open(uniq_filename + ".ceps", 'w') as f:
    for d in data:
        for e in d:
            f.write(str(e) + "\n")
    #f.write(data2)
print("Read buffer")

