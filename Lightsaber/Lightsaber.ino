

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


// ---------------------------- PINS -------------------------------
#define GYRO_PIN 2
#define BLADE_PIXEL_PIN 3
#define CONTROL_PIXEL_PIN 4
#define BLADE_BUTTON_PIN 5
#define BLADE_BUTTON_LED_PIN 6
#define CHARACTER_BUTTON_PIN 7
#define SOUND_PIN 8
#define SPEAKER_PIN 9

// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1             // Show debug information if set to 1
#define BLADE_LEDS 90       // Number of neopixel leds for the blade
#define PULSE_ALLOW 1       // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20       // pulse amplitude
#define PULSE_DELAY 30      // delay between pulses
#define VOLUME 6            // Maximum volume (should probably not be changed if volume is controlled with potentiometer)

// ---------------------------- VARIABLES -------------------------------
CRGB pixels[BLADE_LEDS];    // Holds blade neopixels
TMRpcm tmrpcm;              // Define tmrpcm used for sound playback

int pixelsPerSide = BLADE_LEDS / 2; // The neopixel strip is folded in half to light up both sides of the blade

Button bladeButton(BLADE_BUTTON_PIN); // Button controlling the status of the blade
Button characterButton(CHARACTER_BUTTON_PIN); // Button for selecting character
Button soundButton(SOUND_PIN);  // Button to make a random sound from the current characters sound bank

byte red, green, blue, redOffset, greenOffset, blueOffset; // RGB color information and offsets
unsigned long PULSE_timer; // Time since last pulse
int PULSEOffset;

float k = 0.2; 
bool bladeActive;

// --------------------------------- SOUNDS ----------------------------------
const char strike1[] PROGMEM = "SK1.wav";
const char strike2[] PROGMEM = "SK2.wav";
const char strike3[] PROGMEM = "SK3.wav";
const char strike4[] PROGMEM = "SK4.wav";
const char strike5[] PROGMEM = "SK5.wav";
const char strike6[] PROGMEM = "SK6.wav";
const char strike7[] PROGMEM = "SK7.wav";
const char strike8[] PROGMEM = "SK8.wav";

const char* const strikes[] PROGMEM  = {
  strike1, strike2, strike3, strike4, strike5, strike6, strike7, strike8
};

int strike_time[8] = {779, 563, 687, 702, 673, 661, 666, 635};

const char strike_s1[] PROGMEM = "SKS1.wav";
const char strike_s2[] PROGMEM = "SKS2.wav";
const char strike_s3[] PROGMEM = "SKS3.wav";
const char strike_s4[] PROGMEM = "SKS4.wav";
const char strike_s5[] PROGMEM = "SKS5.wav";
const char strike_s6[] PROGMEM = "SKS6.wav";
const char strike_s7[] PROGMEM = "SKS7.wav";
const char strike_s8[] PROGMEM = "SKS8.wav";

const char* const strikes_short[] PROGMEM = {
  strike_s1, strike_s2, strike_s3, strike_s4,
  strike_s5, strike_s6, strike_s7, strike_s8
};
int strike_s_time[8] = {270, 167, 186, 250, 252, 255, 250, 238};

const char swing1[] PROGMEM = "SWS1.wav";
const char swing2[] PROGMEM = "SWS2.wav";
const char swing3[] PROGMEM = "SWS3.wav";
const char swing4[] PROGMEM = "SWS4.wav";
const char swing5[] PROGMEM = "SWS5.wav";

const char* const swings[] PROGMEM  = {
  swing1, swing2, swing3, swing4, swing5
};
int swing_time[8] = {389, 372, 360, 366, 337};

const char swingL1[] PROGMEM = "SWL1.wav";
const char swingL2[] PROGMEM = "SWL2.wav";
const char swingL3[] PROGMEM = "SWL3.wav";
const char swingL4[] PROGMEM = "SWL4.wav";

const char* const swings_L[] PROGMEM  = {
  swingL1, swingL2, swingL3, swingL4
};
int swing_time_L[8] = {636, 441, 772, 702};

char BUFFER[10];

// --------------------------------- CHARACTERS ----------------------------------
// 0 - Darth Vader, Red
// 1 - Kylo Ren, Red
// 2 - Darth Sidious, Red
// 3 - Luke Skywalker, Blue
// 4 - Yoda, Green
// 5 - Obi Wan kenobi, Blue
// 6 - Mace Windu, Purple
// ---------------------------------------------------------------------------

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
  if (DEBUG) {
    Serial.println(msg);
  }
}
