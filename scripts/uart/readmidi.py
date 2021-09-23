import serial
import datetime

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM5'
print(ser)
ser.open()

data = []

last_midi = 0

for i in range(1000):
    new_data = ser.read(3)
    data.append(new_data)
    #print(new_data[0], new_data[1], new_data[2])
    if (new_data[1] != last_midi):
        last_midi = new_data[1]
        print(last_midi)



# #exit()
uniq_filename = str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '.')
with open("mididata/" + uniq_filename + ".mididat", 'w') as f:
    for i in data:
        f.write(str(i[1]) + "\n")
    #f.write(data2)
print("Read 1000 midi messages")

