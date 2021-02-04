

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

#define PULSE_ALLOW 1       // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20       // pulse amplitude
#define PULSE_DELAY 30      // delay between pulses


CRGB pixels[BLADE_LEDS];
TMRpcm tmrpcm;

const bool debug = true;

int pixelsPerSide = BLADE_LEDS / 2;
int buttonState = 0;
int extended = 0;

Button bladeButton(BLADE_BUTTON_PIN);
Button characterButton(CHARACTER_BUTTON_PIN);
Button soundButton(SOUND_PIN);

byte nowColor, red, green, blue, redOffset, greenOffset, blueOffset;
unsigned long PULSE_timer;
bool bladeActive;
int PULSEOffset;

float k = 0.2;

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

  setColor(0);

}

void setColor(byte color){
  switch (color) {
    // 0 - red
    case 0:
      red = 255;
      green = 0;
      blue = 0;
      break;
  }
}

void loop() {
  bladePulse();
  //getFreq();
  checkInput();
  //tick();
  //strike();
  //swing();
  
}

void bladePulse(){
    if (PULSE_ALLOW && bladeActive && (millis() - PULSE_timer > PULSE_DELAY)) {
      PULSE_timer = millis();
      PULSEOffset = PULSEOffset * k + random(-PULSE_AMPL, PULSE_AMPL) * (1 - k);
      if (nowColor == 0) PULSEOffset = constrain(PULSEOffset, -15, 5);
      redOffset = constrain(red + PULSEOffset, 0, 255);
      greenOffset = constrain(green + PULSEOffset, 0, 255);
      blueOffset = constrain(blue + PULSEOffset, 0, 255);
      setAllBladePixels(redOffset, greenOffset, blueOffset);
    }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  pixels[Pixel].r = red;
  pixels[Pixel].g = green;
  pixels[Pixel].b = blue;
}

void setAllBladePixels(byte red, byte green, byte blue) {
  for (int i = 0; i < BLADE_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
}

void checkInput() {
  if (bladeButton.toggled()) {
     if (bladeButton.read() == Button::PRESSED) {
      if (bladeActive != true) {
        animateExtend();
      }
      else {
        animateRetract();
      }
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
  bladeActive = true;

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
  bladeActive = false;
}

// Print to serial out only if debug is enabled
void serialPrint(String msg) {
  if (debug) {
    Serial.println(msg);
  }
}
