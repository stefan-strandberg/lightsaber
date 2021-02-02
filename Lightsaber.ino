

/*
  ### SD CARD ###
  SD card attached to SPI bus as follows:
    * MOSI - pin 11
    * MISO - pin 12
    * CLK - pin 13
    * CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  ### GYRO ###
  Gyro attached to I2C bus

  ### CONTROL BUTTONS ###
  * Led button extend and retract blade
  * Control button to change blade color/character
  * Control button to make random character noice


*/

#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <TMRpcm.h>
#include "Button.h"
#include "FastLED.h"

#define BLADE_LEDS 90

#define GYRO_PIN 2
#define BLADE_PIXEL_PIN 3
#define CONTROL_PIXEL_PIN 4
#define BLADE_BUTTON_PIN 5
#define BLADE_BUTTON_LED_PIN 6
#define CHARACTER_BUTTON_PIN 7
#define SOUND_PIN 8
#define SPEAKER_PIN 9

#define VOLUME 6


CRGB pixels[BLADE_LEDS];
TMRpcm tmrpcm;

const bool debug = true;

int pixelsPerSide = BLADE_LEDS / 2;
int buttonState = 0;
int extended = 0;

Button bladeButton(BLADE_BUTTON_PIN);
Button characterButton(CHARACTER_BUTTON_PIN);
Button soundButton(SOUND_PIN);

const int buttonPin = 7;


void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  serialPrint("Initializing SD card...");

  if (!SD.begin(3)) {
    //TODO: Play sound to indicate failure
    serialPrint("initialization failed!");
    while (1);
  }
  serialPrint("SD card initialized.");
  
  //FastLED.addLeds<NEOPIXEL, BLADE_PIXEL_PIN>(pixels, BLADE_LEDS);

  tmrpcm.speakerPin = SPEAKER_PIN;
  tmrpcm.setVolume(VOLUME);
  tmrpcm.quality(1);

  bladeButton.begin();
  characterButton.begin();
  soundButton.begin();

 
}

// Print to serial out only if debug is enabled
void serialPrint(String msg) {
  if (debug) {
    Serial.println(msg);
  }
}

void loop() {

  checkInput();
  
}

void checkInput() {

  if (bladeButton.toggled()) {
     if (bladeButton.read() == Button::PRESSED) {
      // TODO: Check if blade is extended or not
      // Should also control the led
     } 
  }

}

void animateExtend() {
 // pixels[0] = CRGB::Red; FastLED.show();

  for(int dot = 0; dot <= (BLADE_LEDS / 2 - 1); dot++) { 

      pixels[dot] = CRGB::Red;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Red;
      Serial.println(dot);
      // clear this led for the next time around the loop
      //leds[dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }

}
  

void animateRetract() {
  for(int dot = (BLADE_LEDS / 2 - 1); dot >= 0; dot--) { 
     Serial.println(dot);
      pixels[dot] = CRGB::Black;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Black;
      
      // clear this led for the next time around the loop
      //leds[dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }
}
