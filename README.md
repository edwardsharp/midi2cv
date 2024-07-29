# MIDI2CV

just some musing on a midi to cv (control voltage) converter for eurorack.

it's basically 4 mcp4728 DACs to provide 16 channels of control voltages (0-5v). there's a midi interface for 3.5mm TRS input which uses [adafruit's midiwing kit](https://www.adafruit.com/product/4740). all powered by a rp2040 microcontroller (in this case an[adafruit feather](https://www.adafruit.com/product/4884))

and finally, a [circuit python](https://circuitpython.org/) script to coble it all together. for each midi channel, there's an output voltage for the single note (monophonic) being played, it's scaled from 0-127 (midi note) to 0-5v (roughly 12bit resolution).

### dev notes

make sure you have these libraries:

[adafruit_bus_device](https://github.com/adafruit/Adafruit_CircuitPython_BusDevice)

[adafruit_mcp4728.mpy](https://github.com/adafruit/Adafruit_MCP4728)

[adafruit_midi](https://github.com/adafruit/Adafruit_CircuitPython_MIDI)

![midi2cv prototype](midi2cv.png)

---

made with 🖤 in NYC
