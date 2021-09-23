#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import glob
import os
import struct

fs = 48828

cp_framesize = 1250

max_freq = 1200 #yea.h
min_freq = 80 #yea.h

min_time = 1. / max_freq #yea.h
max_time = 1. / min_freq #yea.h

min_index = math.ceil(min_time * fs)
max_index = math.floor(max_time* fs)


t = [0] * cp_framesize #yea.c unitinitialized array


for i in range(cp_framesize): 
    t[i] = i / fs               #yea.c init_vars check 

def quantize_frequency_to_MIDI(in_freq): #yessir.c
    n = 0
    n = in_freq
    n = n/440
    n = np.log2(n)
    n = n * 12
    n = n + 69
    n = round(n)
    n = int(n)
    return n


def argmax(l):
    maximum = l[0]
    maxindex = 0
    for i in range(len(l)):
        if l[i] > maximum:
            maximum = l[i]
            maxindex = i
    return maxindex



if (len(sys.argv) == 1):
    list_of_files = glob.glob('uart/bufferdata/*.dat') # * means all if need specific format then *.csv
    #print(list_of_files)
    latest_file = max(list_of_files, key=os.path.getctime)
    fname = latest_file
else:
    fname = "./" + sys.argv[1]

print("Opening " + fname)

origsignal = []

with open(fname, "rb") as f:
    data = f.read()

for i in range(0, len(data), 4):
    origsignal.append(struct.unpack(">i", data[i:i+4]))

origsignal = [x[0] for x in origsignal] # flatten origsignal

for i in range(0,len(origsignal), cp_framesize):
    this_buffer = origsignal[i: i+cp_framesize]
    spectrum = np.fft.fft(this_buffer)

    mag = np.abs(spectrum)**2
    logmagsquared = np.log((np.abs(spectrum))**2)

    ceps = np.abs(np.fft.fft(mag))**2
    index = argmax(ceps[min_index:max_index]) + min_index

    figure, axes = plt.subplots(nrows=2, ncols=2)

    print(this_buffer[0])

    axes[0,1].set_xlim(-0.025, 0.025)
    axes[1,0].set_xlim(-0.025, 0.025)
    axes[1,1].set_xlim(min_index, max_index)
    axes[0,0].plot(origsignal)
    axes[0,1].plot(np.fft.fftfreq(len(spectrum)),spectrum)
    axes[1,0].plot(np.fft.fftfreq(len(mag)),mag)
    axes[1,1].plot(range(min_index,max_index),ceps[min_index:max_index])


    goal_freq = 82.4068892282
    goal_time = goal_freq**-1
    goal_index = round(goal_time * fs)


    print("Frequency Range = (%f, %f)" % (t[max_index]**-1, t[min_index]**-1))
    print("Goal Frequency = %f" % goal_freq)
    print("Read Frequency = %f" % t[index]**-1)
    print("Time Range = (%f, %f)" % (t[min_index], t[max_index]))
    print("Goal Time = %f " % goal_time)
    print("Read Time = %f" % t[index])
    print("Index Range = (%d, %d)" % (min_index, max_index))
    print("Goal Index = %d" % goal_index)
    print("Read Index = %d" % index)
    # print(t[min_index], t[max_index])
    # print(index)
    # print(t[index])

    print("MIDI Range: (%d, %d)" % (quantize_frequency_to_MIDI(t[max_index]**-1),  quantize_frequency_to_MIDI(t[min_index]**-1)))
    print("Goal MIDI: %d" % quantize_frequency_to_MIDI(goal_freq))
    print("Read MIDI: %d" % quantize_frequency_to_MIDI(t[index]**-1))
    plt.show()



