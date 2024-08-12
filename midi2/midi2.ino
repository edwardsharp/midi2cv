/* midi2cv
 * 
 * an arduino sketch for converting usb (host!) midi into control voltage outputs.
 * it will output a note, velocity, and gate cv for 8 incoming midi channels.
 *
 * this is made for the adafruit rp2040 feather usb host
 * see: https://www.adafruit.com/product/5723
 *
 * this leverages a couple useful libraries for handling usb host mode (i.e. non-device mode, which is tricky
 * on the rp2040 microcontroller) which is a second usb port on the rp2040. 
 * 1. pio (programmable io) for usb host communication 
 *    this is based on some bit-banging code via pio_usb.h, see also: https://github.com/sekigon-gonnoc/Pico-PIO-USB
 *    IMPORTANT NOTE: use version 0.5.3 of the library (0.6.0 didn't work!)
 * 2. usb host midi handling is conveniently abstracted via the ez_usb_midi_host library
 *    see more here: https://github.com/rppicomidi/EZ_USB_MIDI_HOST/
 * 
 * finally there's two other ICs for control voltage outputs:
 * 1. a 74HC595 shift register controls 8 LEDs and 8 5v CV gate outputs 
 * 2. four MCP4728 DACs control the final 16 channels of 0-5v CV outputs
 *    8 for midi note output and 8 for midi note velocity output.
 *    IMPORTANT NOTE: the MCP4728 DACs are controlled via another microcontroller. i ran into problems
 *    where running usb midi AND sending i2c to multiple DACs caused crashes :(
 *    so this program just sends UART Serial messages to another microcontroller that is connected to 4 MCP4728 DACs. 
*/

// pio-usb for rp2040 host usb stuff
#include "pio_usb.h"
#define HOST_PIN_DP 16  // Pin used as D+ for rp2040 usb host, D- pin is just +1 pin (so 16, 17)

#include "Adafruit_TinyUSB.h"
#define LANGUAGE_ID 0x0409  // english

// usb midi stuff
#include "EZ_USB_MIDI_HOST.h"
Adafruit_USBH_Host USBHost;
USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST
// MIDI_NAMESPACE::DefaultSettings
// struct MyMidiHostSettingsDefault : public MidiHostSettingsDefault {
//   static const unsigned SysExMaxSize = 64;  // for MIDI Library
//   static const unsigned MidiRxBufsize = RPPICOMIDI_EZ_USB_MIDI_HOST_GET_BUFSIZE(SysExMaxSize);
//   static const unsigned MidiTxBufsize = RPPICOMIDI_EZ_USB_MIDI_HOST_GET_BUFSIZE(SysExMaxSize);
// };
RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault);
static uint8_t midiDevAddr = 0;

// usb device descriptor
tusb_desc_device_t desc_device;

// cable is basically when midi devices have multiple composite devices
// e.g. the launchpad x shows up as 2 midi devices, the first is for DAW integration
// and the second is the raw midi typically expected
// so track here, when mounting usb midi device, set this to the last cable number for a device
uint8_t cable = 0;

// 74HC595 shift register stuff
// clock pin connected to SH_CP of 74HC595 (YELLOW wire)
int clockPin = 13;
// latch pin connected to ST_CP of 74HC595 (GREEN wire)
int latchPin = 12;
// data pin connected to DS of 74HC595 (BLUE wire)
int dataPin = 11;

// state stuff
// 8 channels of gate state
bool gates[8] = { false, false, false, false, false, false, false, false };
// note and velocity buffer state (12 bit numbers for DAC voltages)
int note_voltz[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int velo_voltz[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// poly -> monophonic note tracking
// (so turn velocity and gate off when all notes for a channel are off)
int notes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// so for the polyend tracker, the midi out note range is p much note 36-83 (there are 48 padz).
// note: if there's an incoming note outside this range, the scale fn will mutate this int
// e.g. if midi note `2` is received then note_min will get set to `2`, see: note2voltage() fn.
int note_min = 36;
int note_max = 83;
// max 12bit int for DAC voltages 
int FULL_VREF_VALUE = 4095;

void setup() {
  Serial.begin(115200);  // init serial debug monitor

  // initialize UART on Serial1 (for sending note_voltz and velo_voltz)
  // note: 1000000 is 1mbps! 9600 is too slow :/
  Serial1.begin(1000000);

  // while (!Serial) delay(10);  // wait for native usb
  Serial.println("midi2cv hiiiiii!");

  // check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    while (!Serial) delay(10);  // wait for native usb
    Serial.printf("ERROR: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while (1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;

  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  // midi stuff
  usbhMIDI.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);

  // shift register setup
  pinMode(latchPin, OUTPUT);

  // pull up PIN_5V_EN pin to turn on 5v on usb host port
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, 1);

  // blink the shift register ledz on startup
  initShiftRegLedz();
}

void loop() {
  USBHost.task();
  usbhMIDI.readAll();
  // usbhMIDI.writeFlushAll();
}

// void setup1() {}

// void loop1() {}

// scale midi note velocity (in this case: to 12 bit dac voltage)
int velo2voltage(int note) {
  int in_min = 0;
  int in_max = 127;
  int out_min = 0;
  int out_max = FULL_VREF_VALUE;
  return round((note - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int note2voltage(int note) {
  // so a bit of an easter egg here
  // BUT
  // this will auto scale midi notes to the widest range
  // ...is this a hysteresis?!
  // dunno, but the first thing to do is: mash the lowest note && the highest note
  if (note > note_max) {
    note_max = note;
  }
  if (note < note_min) {
    note_min = note;
  }

  int out_min = 0;
  int out_max = FULL_VREF_VALUE;
  return round((note - note_min) * (out_max - out_min) / (note_max - note_min) + out_min);
}

// midi callbackz
/* CONNECTION MANAGEMENT */
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables) {
  Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
  midiDevAddr = devAddr;
  cable = nInCables - 1;
  registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr) {
  Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
  midiDevAddr = 0;
}
/* ERRORZ :( */
static void onMidiError(int8_t errCode) {
  Serial.printf("MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse" : "",
                (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
                (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx" : "");
}

static void onMidiInWriteFail(uint8_t devAddr, uint8_t cable, bool fifoOverflow) {
  if (fifoOverflow)
    Serial.printf("Dev %u cable %u: MIDI IN FIFO overflow\r\n", devAddr, cable);
  else
    Serial.printf("Dev %u cable %u: MIDI IN FIFO error\r\n", devAddr, cable);
}

/* NOTE HANDLING */
static void onNoteOff(Channel channel, byte note, byte velocity) {
  Serial.printf("C%u: Note off#%u v=%u\r\n", channel, note, velocity);
  // zero-indexed channel:
  int ch = channel - 1;
  notes[ch] -= 1;

  if (notes[ch] <= 0) {
    // #TODO: why did i use `velocity > 0` 🤔
    // gates[ch] = velocity > 0;
    gates[ch] = 0;
    velo_voltz[ch] = 0;

    shiftOut();
    voltzOut();
  }
}

static void onNoteOn(Channel channel, byte note, byte velocity) {
  Serial.printf("C%u: Note on#%u v=%u\r\n", channel, note, velocity);
  int note_v = note2voltage(note);
  int velocity_v = velo2voltage(velocity);
  // zero-index channel:
  int ch = channel - 1;
  notes[ch] += 1;
  gates[ch] = velocity > 0;
  velo_voltz[ch] = velo2voltage(velocity);
  note_voltz[ch] = note2voltage(note);

  shiftOut();
  voltzOut();
}

static void registerMidiInCallbacks() {
  auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, cable);
  if (intf == nullptr)
    return;
  intf->setHandleNoteOn(onNoteOn);    // 0x90
  intf->setHandleNoteOff(onNoteOff);  // 0x80
  // intf->setHandleAfterTouchPoly(onPolyphonicAftertouch);  // 0xA0
  // intf->setHandleControlChange(onControlChange);          // 0xB0
  // intf->setHandleProgramChange(onProgramChange);          // 0xC0
  // intf->setHandleAfterTouchChannel(onAftertouch);         // 0xD0
  // intf->setHandlePitchBend(onPitchBend);                  // 0xE0
  // intf->setHandleSystemExclusive(onSysEx);                // 0xF0, 0xF7
  // intf->setHandleTimeCodeQuarterFrame(onSMPTEqf);         // 0xF1
  // intf->setHandleSongPosition(onSongPosition);            // 0xF2
  // intf->setHandleSongSelect(onSongSelect);                // 0xF3
  // intf->setHandleTuneRequest(onTuneRequest);              // 0xF6
  // intf->setHandleClock(onMidiClock);                      // 0xF8
  // // 0xF9 as 10ms Tick is not MIDI 1.0 standard but implemented in the Arduino MIDI Library
  // intf->setHandleTick(onMidiTick);                        // 0xF9
  // intf->setHandleStart(onMidiStart);                      // 0xFA
  // intf->setHandleContinue(onMidiContinue);                // 0xFB
  // intf->setHandleStop(onMidiStop);                        // 0xFC
  // intf->setHandleActiveSensing(onActiveSense);            // 0xFE
  // intf->setHandleSystemReset(onSystemReset);              // 0xFF
  intf->setHandleError(onMidiError);

  auto dev = usbhMIDI.getDevFromDevAddr(midiDevAddr);
  if (dev == nullptr)
    return;
  dev->setOnMidiInWriteFail(onMidiInWriteFail);
}


// shift register stuff
void initShiftRegLedz() {
  // by default all gates are HIGH after power on.
  // blink each one during setup()
  for (int i = 0; i < 9; i++) {
    // if last one, be done
    if (i == 8) {
      gates[7] = false;
      shiftOut();
      return;
    }
    // turn prev off
    if (i > 0) {
      gates[i - 1] = false;
    }
    gates[i] = true;
    shiftOut();
    // ramp the time (because asetheticz)
    delay(i * 33);
  }
}
void shiftOut() {
  // shifts 8 bits out (MSB first),
  // on the rising edge of the clock.
  // clock idles low.
  // internal function setup
  int i = 0;
  int pinState;
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  // clear everything out and
  // prepare shift register for bit shifting
  digitalWrite(latchPin, 0);
  digitalWrite(dataPin, 0);
  digitalWrite(clockPin, 0);

  // NOTICE THAT THIS LOOP IS COUNTING DOWN,
  // so this means that %00000001 or "1" will go through such
  // that it will be pin Q0 that lights.
  for (i = 7; i >= 0; i--) {
    digitalWrite(clockPin, 0);
    if (gates[i]) {
      // set HIGH
      digitalWrite(dataPin, 1);
    } else {
      // set LOW
      digitalWrite(dataPin, 0);
    }
    // register shifts bits on upstroke of clock pin
    digitalWrite(clockPin, 1);
    // zero the data pin after shift to prevent bleed through
    digitalWrite(dataPin, 0);
  }
  // stop shifting
  digitalWrite(clockPin, 0);
  digitalWrite(latchPin, 1);
}

void voltzOut() {
  // first, concatenate the note and velo voltz arrays
  // create a buffer large enough to hold both arrays
  const int bufferSize = sizeof(note_voltz) + sizeof(velo_voltz);
  uint8_t buffer[bufferSize];

  // copy first array into buffer, then copy second array right after the first one
  memcpy(buffer, note_voltz, sizeof(note_voltz));
  memcpy(buffer + sizeof(note_voltz), velo_voltz, sizeof(velo_voltz));

  // start marker
  Serial1.write('<');
  // send the combined buffer
  Serial1.write(buffer, bufferSize);
  // end marker
  Serial1.write('>');
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  Serial.printf("Device attached, address = %d\r\n", daddr);

  // Get Device Descriptor
  tuh_descriptor_get_device(daddr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr) {
  Serial.printf("Device removed, address = %d\r\n", daddr);
}

// everything below is just debug utils for parsing and printing usb device descriptors
// #TODO: could yank all this before being "done".

void print_device_descriptor(tuh_xfer_t *xfer) {
  if (XFER_RESULT_SUCCESS != xfer->result) {
    Serial.printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  Serial.printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  Serial.printf("Device Descriptor:\r\n");
  Serial.printf("  bLength             %u\r\n", desc_device.bLength);
  Serial.printf("  bDescriptorType     %u\r\n", desc_device.bDescriptorType);
  Serial.printf("  bcdUSB              %04x\r\n", desc_device.bcdUSB);
  Serial.printf("  bDeviceClass        %u\r\n", desc_device.bDeviceClass);
  Serial.printf("  bDeviceSubClass     %u\r\n", desc_device.bDeviceSubClass);
  Serial.printf("  bDeviceProtocol     %u\r\n", desc_device.bDeviceProtocol);
  Serial.printf("  bMaxPacketSize0     %u\r\n", desc_device.bMaxPacketSize0);
  Serial.printf("  idVendor            0x%04x\r\n", desc_device.idVendor);
  Serial.printf("  idProduct           0x%04x\r\n", desc_device.idProduct);
  Serial.printf("  bcdDevice           %04x\r\n", desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  Serial.printf("  iManufacturer       %u     ", desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iProduct            %u     ", desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iSerialNumber       %u     ", desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  bNumConfigurations  %u\r\n", desc_device.bNumConfigurations);
}

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void)utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xff;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = _count_utf8_bytes(temp_buf + 1, utf16_len);

  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *)temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t *)temp_buf)[utf8_len] = '\0';

  Serial.printf((char *)temp_buf);
}
