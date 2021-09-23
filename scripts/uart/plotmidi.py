#!/usr/bin/env python3
# Usage: plotmidi [<UART dump>]
# Given a UART dump of MIDI signals (in text not binary), plot them on a histogram
# If 1 arg is given, this is interpreted as the data to plot. If 0 are given, we use the last-created file in with the ext .mididat
# Matthew Bremer and Mikey Brewer 2020

import sys
from matplotlib import pyplot as plt
import glob
import os
import struct


if (len(sys.argv) == 1):
    list_of_files = glob.glob('mididata/*.mididat') # * means all if need specific format then *.csv
    #print(list_of_files)
    latest_file = max(list_of_files, key=os.path.getctime)
    fname = latest_file
else:
    fname = "./" + sys.argv[1]

print("Opening " + fname)

data = []

with open(fname, 'r') as f:
    data = [int(x) for x in f.read().split("\n") if x != '']

print("Read " + str(len(data)) + " bytes")

plt.xticks(range(min(data), max(data) + 1))
plt.hist(data, bins=range(min(data), max(data) + 1))
plt.show()
