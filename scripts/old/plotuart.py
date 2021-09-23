import sys
from matplotlib import pyplot as plt
import glob
import os
import struct


if (len(sys.argv) == 1):
    list_of_files = glob.glob('./*.dat') # * means all if need specific format then *.csv
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

newdata = bytearray(data)
ints = []

for i in range(0, len(data), 4):
    newdata[i+0 : i+4] = data[i+1], data[i+0], data[i+3], data[i+2]
    ints.append(struct.unpack(">i", newdata[i:i+4]))

left = [ints[i] for i in range(len(ints)) if i % 2 == 0]
right = [ints[i] for i in range(len(ints)) if i % 2 == 1]

figure, axes = plt.subplots(nrows=2, ncols=1)

axes[0].scatter(list(range(len(left))),left)
axes[1].scatter(list(range(len(right))),right, marker='.',cmap='magma')


jumps = [right[i][0] - right[i-1][0] for i in range(len(right))]

jump_indexes = [i for i in range(len(jumps)) if jumps[i] > 8e8 or jumps[i] < -8e8]

diffs = [jump_indexes[i] - jump_indexes[i-1] for i in range(1, len(jump_indexes))]
#plt.figure()
#plt.plot(jumps)
#print(jump_indexes)

#plt.figure()
#plt.plot(ints)
plt.show()
