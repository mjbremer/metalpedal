import matplotlib.pyplot as plt
import numpy as np
import math

# Fs = 44100
# f = 100

# x = np.arange(sample)
# y = np.sin(2 * np.pi * f * x / Fs) * 2147483646 / 2

# print(len(y))

# writedata = [int(i) for i in y]

# with open('signal.dat', 'wb') as f:
#     for i in writedata:
#         f.write((i).to_bytes(4, byteorder = "little", signed = True))

# #plt.plot(x, y)
# #plt.show()

f = 440
fs = 48828

NUM_BUFFERS = 1000
samplesperperiod = int(fs / f)

nharm = 1000
listlen = samplesperperiod

coefs = [((2147483646/math.pi) * ((-1)**k) / k) for k in range(1,nharm+1)]
wavelist = []
for i in range(0, listlen):
    sample = 0
    for k in range(0, nharm):
        sample += coefs[k] * math.sin(2 * math.pi * i * (k+1) / listlen)
    wavelist.append(int(sample))

sample = 1250*NUM_BUFFERS

writedata = wavelist * math.ceil(sample/samplesperperiod)

with open('signal.dat', 'wb') as f:
    for i in writedata[:sample]:
        f.write((i).to_bytes(4, byteorder = "little", signed = True))



figure, axes = plt.subplots(nrows=2)

axes[0].plot(wavelist)
axes[1].plot(writedata)
plt.savefig("signal.png")

plt.show()
