# MIDI2CV

💁‍♀️ ohey! check out where i'm at with rev2 (now with more rust!) https://github.com/edwardsharp/midi2cv/pull/1

just some musing on a midi to cv (control voltage) converter for eurorack.

it's basically 4 mcp4728 DACs to provide 16 channels of control voltages (0-5v). there's a midi interface for 3.5mm TRS input which uses [adafruit's midiwing kit](https://www.adafruit.com/product/4740). all powered by a rp2040 microcontroller (in this case an[adafruit feather](https://www.adafruit.com/product/4884) or maybe a pico, dunno for sure yet (see also: [pico.pinout.xyz](https://pico.pinout.xyz/)))

so for each midi channel (ch 0-7), there's an output voltage for a single note (monophonic) being played, it's velocity, and a gate.

note and velocity voltages are scaled from 0-127 (midi note) to 0-5v (roughly 12bit resolution); gates are 5v.

### dev notes

make sure you have these libraries:

`cargo build`

plug in the pico pi (while holding botsel button), then:

`cargo run --release`

---

made with 🖤 in NYC
