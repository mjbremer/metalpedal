import serial
import datetime

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM4'
print(ser)
ser.open()


data = ser.read(int(1250*4)) # read buffer
print("Read buffer")
guess = ser.read(3) # read associated guess
print("Read guess")

uniq_filename = str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '.')

buffer_fname = "bufferdata/" + uniq_filename + ".dat"
midi_fname = "mididata/" + uniq_filename + ".mididat"

with open(buffer_fname, 'wb') as f:
    f.write(data)

with open(midi_fname, 'w') as f:
    f.write(str(guess[1]) + "\n")


