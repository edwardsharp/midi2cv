# midi2cv

import board
import digitalio
import busio
import adafruit_midi
import adafruit_mcp4728
from adafruit_midi.note_off import NoteOff
from adafruit_midi.note_on import NoteOn

# import time

tx = board.GP0
rx = board.GP1
uart = busio.UART(tx, rx, baudrate=31250, timeout=0.001)  # init UART

midi = adafruit_midi.MIDI(
    midi_in=uart,
    midi_out=uart,
    # listen on all channels
    in_channel=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15),
    out_channel=(0),
    debug=False,
)

# DAC
# scl=board.GP3, sda=board.GP2
i2c = busio.I2C(board.GP3, board.GP2)

# i2c addresses for 4 DACs
# note: addresses might vary,
# i programmed 4 DACs to these addresses
dac0 = adafruit_mcp4728.MCP4728(i2c, 0x60)
dac1 = adafruit_mcp4728.MCP4728(i2c, 0x61)
dac2 = adafruit_mcp4728.MCP4728(i2c, 0x62)
dac3 = adafruit_mcp4728.MCP4728(i2c, 0x63)

# could change Vref like:
# dac0.channel_a.vref = adafruit_mcp4728.Vref.VDD

# set all channels to be 2x gain (so like ~0-4v)
dac0.channel_a.gain = 2
dac0.channel_b.gain = 2
dac0.channel_c.gain = 2
dac0.channel_d.gain = 2

dac1.channel_a.gain = 2
dac1.channel_b.gain = 2
dac1.channel_c.gain = 2
dac1.channel_d.gain = 2

dac2.channel_a.gain = 2
dac2.channel_b.gain = 2
dac2.channel_c.gain = 2
dac2.channel_d.gain = 2

dac3.channel_a.gain = 2
dac3.channel_b.gain = 2
dac3.channel_c.gain = 2
dac3.channel_d.gain = 2

# so DACs are 12-bit but py code takes 16-bit int
# any-which-way this is the biggest int we can use
FULL_VREF_VALUE = 65535

# FULL_VREF_RAW_VALUE = 4095
# so could do like:
# dac0.channel_b.raw_value = int(FULL_VREF_RAW_VALUE)

# scale MIDI notes to to DAC voltage
def note2Voltage(note):
    in_min = 0
    in_max = 127
    out_min = 0
    out_max = FULL_VREF_VALUE
    return int((note - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def note2DAC(channel, note):
    # print("note2dac v:", note2Voltage(note))
    if channel == 0:
        dac0.channel_a.value = note2Voltage(note)
    elif channel == 1:
        dac0.channel_b.value = note2Voltage(note)
    elif channel == 2:
        dac0.channel_c.value = note2Voltage(note)
    elif channel == 3:
        dac0.channel_d.value = note2Voltage(note)
    elif channel == 4:
        dac1.channel_a.value = note2Voltage(note)
    elif channel == 5:
        dac1.channel_b.value = note2Voltage(note)
    elif channel == 6:
        dac1.channel_c.value = note2Voltage(note)
    elif channel == 7:
        dac1.channel_d.value = note2Voltage(note)
    elif channel == 8:
        dac2.channel_a.value = note2Voltage(note)
    elif channel == 9:
        dac2.channel_b.value = note2Voltage(note)
    elif channel == 10:
        dac2.channel_c.value = note2Voltage(note)
    elif channel == 11:
        dac2.channel_d.value = note2Voltage(note)
    elif channel == 12:
        dac3.channel_a.value = note2Voltage(note)
    elif channel == 13:
        dac3.channel_b.value = note2Voltage(note)
    elif channel == 14:
        dac3.channel_c.value = note2Voltage(note)
    elif channel == 15:
        dac0.channel_d.value = note2Voltage(note)


# print("MIDI2CV")
# print("MIDI output channel:", midi.out_channel)
# print("MIDI input channel:", midi.in_channel)


# led
# lil led is GP13
# NEOPIXEL?
led = digitalio.DigitalInOut(board.GP13)
led.direction = digitalio.Direction.OUTPUT

# MIDI2CV
while True:
    msg_in = midi.receive()
    if isinstance(msg_in, NoteOn) and msg_in.velocity != 0:
        # print("note_on:", msg_in.note, "ch:", msg_in.channel)
        led.value = True
        note2DAC(msg_in.channel, msg_in.note)
    elif (
        isinstance(msg_in, NoteOff)
        or isinstance(msg_in, NoteOn)
        and msg_in.velocity == 0
    ):
        # print("note_off", msg_in.note, " ch:", msg_in.channel)
        led.value = False


# on/off blinky TEST MODE:
# while True:
#     led.value = True
#     dac0.channel_a.value = FULL_VREF_VALUE
#     dac0.channel_b.value = FULL_VREF_VALUE
#     dac0.channel_c.value = FULL_VREF_VALUE
#     dac0.channel_d.value = FULL_VREF_VALUE

#     dac1.channel_a.value = FULL_VREF_VALUE
#     dac1.channel_b.value = FULL_VREF_VALUE
#     dac1.channel_c.value = FULL_VREF_VALUE
#     dac1.channel_d.value = FULL_VREF_VALUE

#     dac2.channel_a.value = FULL_VREF_VALUE
#     dac2.channel_b.value = FULL_VREF_VALUE
#     dac2.channel_c.value = FULL_VREF_VALUE
#     dac2.channel_d.value = FULL_VREF_VALUE

#     dac3.channel_a.value = FULL_VREF_VALUE
#     dac3.channel_b.value = FULL_VREF_VALUE
#     dac3.channel_c.value = FULL_VREF_VALUE
#     dac3.channel_d.value = FULL_VREF_VALUE

#     time.sleep(1)
#     led.value = False

#     dac0.channel_a.value = 0
#     dac0.channel_b.value = 0
#     dac0.channel_c.value = 0
#     dac0.channel_d.value = 0

#     dac1.channel_a.value = 0
#     dac1.channel_b.value = 0
#     dac1.channel_c.value = 0
#     dac1.channel_d.value = 0

#     dac2.channel_a.value = 0
#     dac2.channel_b.value = 0
#     dac2.channel_c.value = 0
#     dac2.channel_d.value = 0

#     dac3.channel_a.value = 0
#     dac3.channel_b.value = 0
#     dac3.channel_c.value = 0
#     dac3.channel_d.value = 0

#     time.sleep(1)
