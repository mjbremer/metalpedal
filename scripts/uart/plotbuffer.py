import sys
from matplotlib import pyplot as plt
import glob
import os
import struct


if (len(sys.argv) == 1):
    list_of_files = glob.glob('bufferdata/*.dat') # * means all if need specific format then *.csv
    #print(list_of_files)
    latest_file = max(list_of_files, key=os.path.getctime)
    fname = latest_file
else:
    fname = "./" + sys.argv[1]

print("Opening " + fname)

data = []

with open(fname, 'rb') as f:
    data = f.read()

print("Read " + str(len(data)) + " bytes")

ints = []

for i in range(0, len(data), 4):
    ints.append(struct.unpack("<i", data[i:i+4]))

#print("{" + ",".join([str(i[0]) for i in ints]))
plt.plot(ints)
plt.show()
