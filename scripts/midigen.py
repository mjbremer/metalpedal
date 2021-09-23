#!/usr/bin/env python3
# Usage: midigen.py <MIDI FILE> <OUTPUT FILE>
# Given a MIDI file, generate a corresponding function using MIDI_On, MIDI_Off, and HAL_Delay functions
# Matthew Bremer and Mikey Brewer 2020
from mido import MidiFile
from sys import argv

if (len(argv) < 3):
    print("Usage: midigen.py <MIDI FILE> <OUTPUT FILE>")
    exit()

midi_file = argv[1]
output_file = argv[2]



program = ""

for msg in MidiFile(midi_file):
    if msg.type in ('note_on', 'note_off'):
        print(msg)
        if (msg.time != 0):
            delaystr = "HAL_Delay(%u);\n" % int(round(msg.time * 1000))
            program += delaystr
            print(delaystr)
        
        argstr = "(%d, %d, %d)" % (msg.channel, msg.note, msg.velocity)
        print(argstr)

        if (msg.type == 'note_on'):
            func = "MIDIon"
        else:
            func = "MIDIoff"
        
        cmdstr = func + argstr + ";\n"

        program += cmdstr
with open(output_file, "w") as f:
    f.write(program)
