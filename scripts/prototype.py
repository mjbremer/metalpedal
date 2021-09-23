#!/usr/bin/env python3
"""Plot the live microphone signal(s) with matplotlib.
Matplotlib and NumPy have to be installed.
"""
import argparse
import queue
import sys

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import math
import sounddevice as sd
from time import sleep


def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text


parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    '-l', '--list-devices', action='store_true',
    help='show list of audio devices and exit')
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)
parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    parents=[parser])
parser.add_argument(
    'channels', type=int, default=[1], nargs='*', metavar='CHANNEL',
    help='input channels to plot (default: the first)')
parser.add_argument(
    '-d', '--device', type=int_or_str,
    help='input device (numeric ID or substring)')
parser.add_argument(
    '-w', '--window', type=float, default=200, metavar='DURATION',
    help='visible time slot (default: %(default)s ms)')
parser.add_argument(
    '-i', '--interval', type=float, default=30,
    help='minimum time between plot updates (default: %(default)s ms)')
parser.add_argument(
    '-b', '--blocksize', type=int, help='block size (in samples)')
parser.add_argument(
    '-r', '--samplerate', type=float, help='sampling rate of audio device')
parser.add_argument(
    '-n', '--downsample', type=int, default=10, metavar='N',
    help='display every Nth sample (default: %(default)s)')
args = parser.parse_args(remaining)
if any(c < 1 for c in args.channels):
    parser.error('argument CHANNEL: must be >= 1')
mapping = [c - 1 for c in args.channels]  # Channel numbers start with 1



# YUCKY PARSING ^^^^^

#############################################################################

# COOL PROCESSING 


##TODO
##Replace np.abs with normal abs
##Add MIDI Output
##Add Attack detection
##
##
##FUTURE
##Multi-peak detection for polyphony

fs = args.samplerate #yea.h

cp_framesize = int(fs/4) #yea.h
cp_buffer = [0] * cp_framesize
cp_pointer = 0 

max_freq = 1200 #yea.h
min_freq = 80 #yea.h

min_time = 1. / max_freq #yea.h
max_time = 1. / min_freq #yea.h

min_index = math.floor(min_time * fs)
max_index = math.ceil(max_time* fs)

midi_note_names = ["LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","A0","A#0/Bb0","B0","C1","C#1/Db1","D1","D#1/Eb1","E1","F1","F#1/Gb1","G1","G#1/Ab1","A1","A#1/Bb1","B1","C2","C#2/Db2","D2","D#2/Eb2","E2","F2","F#2/Gb2","G2","G#2/Ab2","A2","A#2/Bb2","B2","C3","C#3/Db3","D3","D#3/Eb3","E3","F3","F#3/Gb3","G3","G#3/Ab3","A3","A#3/Bb3","B3","C4 (middle C)","C#4/Db4","D4","D#4/Eb4","E4","F4","F#4/Gb4","G4","G#4/Ab4","A4 concert pitch","A#4/Bb4","B4","C5","C#5/Db5","D5","D#5/Eb5","E5","F5","F#5/Gb5","G5","G#5/Ab5","A5","A#5/Bb5","B5","C6","C#6/Db6","D6","D#6/Eb6","E6","F6","F#6/Gb6","G6","G#6/Ab6","A6","A#6/Bb6","B6","C7","C#7/Db7","D7","D#7/Eb7","E7","F7","F#7/Gb7","G7","G#7/Ab7","A7","A#7/Bb7","B7","C8","C#8/Db8","D8","D#8/Eb8","E8","F8","F#8/Gb8","G8","G#8/Ab8","A8","A#8/Bb8","B8","C9","C#9/Db9","D9","D#9/Eb9","E9","F9","F#9/Gb9","G9"]
major_scale = [1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1] #yea.c
last_note = 0
t = [0] * cp_framesize #yea.c unitinitialized array

midi_enabled = [1] * 128
def copy_major_into_enabled(n): #yea.c
    global major_scale
    global midi_enabled
    for i in range(len(major_scale)):
        if (n+i >= len(midi_enabled)):
            return 
        elif (n+i < 0):
            pass
        else: 
            midi_enabled[n+i] = major_scale[i]
    return

def apply_major_scale(n): #been booped
    global major_scale
    global midi_enabled
    assert(n >= 0)
    assert(n <= 11)
    for i in range(-1, 1 + math.ceil((116-n)/12)):
        print(i)
        copy_major_into_enabled(n + 12*i)



for i in range(cp_framesize): 
    t[i] = i / fs               #yea.c init_vars check 

def argmax(l):
    maximum = l[0]
    maxindex = 0
    for i in range(len(l)):
        if l[i] > maximum:
            maximum = l[i]
            maxindex = i
    return maxindex


# https://www.inspiredacoustics.com/en/MIDI_note_numbers_and_center_frequencies
# This function takes an input frequency in Hz in_freq
# and outputs the nearest MIDI note in the following format:
# (midi_freq, midi_n, midi_name) where :
#midi_freq is the frequency of the note in Hz
#midi_n is the MIDI note number
#midi_name is a string with the actual note name, i.e. (F4)
# example: in_freq = 326.0
# return should be (329.62, 64, "E4")
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

def note_name_from_midi_num(n): #yea.c
    return midi_note_names [n]

    
def copy_into_circular_buffer(data):
    global cp_framesize, cp_buffer, cp_pointer
    cp_buffer[cp_pointer] = data
    cp_pointer = (cp_pointer + 1) % cp_framesize

def audio_callback(indata, frames, time, status):
    global last_note
    """This is called (from a separate thread) for each audio block."""
    for x in indata:
        copy_into_circular_buffer(x[0])
    
    spectrum = np.fft.fft(cp_buffer, n=cp_framesize)
    ceps = np.abs(np.fft.ifft(np.log((np.abs(spectrum))**2)))**2
    index = argmax(ceps[min_index:max_index]) + min_index


    new_note = quantize_frequency_to_MIDI(t[index]**-1)
    if (new_note != last_note):
        print(note_name_from_midi_num(new_note))
        last_note = new_note

if __name__ == "__main__":
    stream = sd.InputStream(
        device=args.device, channels=max(args.channels),
        samplerate=fs, callback=audio_callback)
    with stream:
        while(True):
            sleep(1)
    #apply_major_scale(11)
    #for i in range(len(midi_note_names)):
    #   print(midi_note_names[i], midi_enabled[i])
       # make a list from 0 to 11 for notes of the scale that have 0 or 1 corresponding to if you want them to be on or not. do for major and minor (and mode?)
