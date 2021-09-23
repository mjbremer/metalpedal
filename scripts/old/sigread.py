import matplotlib.pyplot as plt
import numpy as np

FRAME_SIZE = 1250
FS = 48828
t = [0] * FRAME_SIZE

for i in range(FRAME_SIZE): 
    t[i] = i / FS        

def argmax(l):
    maximum = l[0]
    maxindex = 0
    for i in range(len(l)):
        if l[i] > maximum:
            maximum = l[i]
            maxindex = i
    return maxindex

data = []
origsignal = []

with open("signal.dat", "rb") as f:
    j = f.read()

for i in range(0, len(j)-3, 4):
    origsignal.append(int.from_bytes(j[i:i+4], byteorder='little', signed=True))

plt.plot(origsignal)
plt.savefig("buffer.png")

#with open("fft.dat", "r") as f:
#    data = f.read()
#
#data = [float(d) for d in  data.split("\n") if d != ""]
#
#index = argmax(data) + 36
#print(t[index]**-1)
#
#figure, axes = plt.subplots(nrows=2, ncols=1)
#
#
#axes[0].plot(origsignal)
#axes[1].plot(data)
plt.show()
