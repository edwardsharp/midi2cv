#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int blinkCounter = 0;

// MCP4728 4-Channel 12-bit I2C DAC stuff
#include <Adafruit_MCP4728.h>
#include <Wire.h>
Adafruit_MCP4728 mcp0;
Adafruit_MCP4728 mcp1;
Adafruit_MCP4728 mcp2;
Adafruit_MCP4728 mcp3;

// dac state
int note_voltz[8];
int velo_voltz[8];

// serial buffer state
bool receiving = false;
int bufferIndex = 0;
const int bufferSize = sizeof(note_voltz) + sizeof(velo_voltz);
uint8_t buffer[bufferSize];

void voltzOut() {
  // Serial.printf("mcp out note_voltz[0]:%u %u %u %u, ", note_voltz[0], velo_voltz[0], note_voltz[1], velo_voltz[1]);
  // Serial.println();

  mcp0.fastWrite(note_voltz[0], velo_voltz[0], note_voltz[1], velo_voltz[1]);
  mcp1.fastWrite(note_voltz[2], velo_voltz[2], note_voltz[3], velo_voltz[3]);
  mcp2.fastWrite(note_voltz[4], velo_voltz[4], note_voltz[5], velo_voltz[5]);
  mcp3.fastWrite(note_voltz[6], velo_voltz[6], note_voltz[7], velo_voltz[7]);
}

void setup() {
  // init neopixel
  pixels.begin();
  pixels.setBrightness(20);  // not so bright

  // to give some time for the the other microcontroller to boot
  // do a lil' startup animation.
  for (int i = 255; i > 0; i--) {
    pixels.fill(pixels.Color(20, i, 20));
    pixels.show();
    delay(4);
  }

  // note: 1000000 is 1mbps! 9600 is too slow :/
  Serial1.begin(1000000);   // init UART on Serial1
  Serial.begin(115200);  // init Serial Monitor

  // note: 0x64 is default address
  // mcp.begin(0x64)
  Wire.setSDA(8);
  Wire.setSCL(9);
  if (!mcp0.begin(0x60)) {
    Serial.println("failed to find mcp0 MCP4728 chip!");
  }
  if (!mcp1.begin(0x61)) {
    Serial.println("failed to find mcp1 MCP4728 chip!");
  }
  if (!mcp2.begin(0x62)) {
    Serial.println("failed to find mcp2 MCP4728 chip!");
  }
  if (!mcp3.begin(0x63)) {
    Serial.println("failed to find mcp3 MCP4728 chip!");
  }
}

void loop() {

  if (blinkCounter > 0) {
    pixels.fill(pixels.Color(20, blinkCounter, 20));
    pixels.show();

    blinkCounter -= 1;
  }

  while (Serial1.available()) {
    char incomingByte = Serial1.read();

    if (incomingByte == '<') {
      // start marker received, begin receiving data
      receiving = true;
      bufferIndex = 0;
    } else if (incomingByte == '>') {
      // end marker received, process the buffer
      if (receiving && bufferIndex == bufferSize) {
        // copy data from buffer into the two local arrays
        memcpy(note_voltz, buffer, sizeof(note_voltz));
        memcpy(velo_voltz, buffer + sizeof(note_voltz), sizeof(velo_voltz));

        voltzOut();

        blinkCounter = 255;

        // debug:
        // Serial.print("received note_voltz: ");
        // for (int i = 0; i < 8; i++) {
        //   Serial.print(note_voltz[i]);
        //   Serial.print(" ");
        // }
        // Serial.println();

        // Serial.print("received velo_voltz: ");
        // for (int i = 0; i < 8; i++) {
        //   Serial.print(velo_voltz[i]);
        //   Serial.print(" ");
        // }
        // Serial.println();
        // Serial.println();
      }

      receiving = false;  // reset receiving state
    } else if (receiving) {
      // collect incoming byte data into the buffer
      if (bufferIndex < bufferSize) {
        buffer[bufferIndex++] = incomingByte;
      }
    }
  }
}