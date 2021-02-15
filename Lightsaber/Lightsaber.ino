
#include <SPI.h>
#include <SdFat.h>
#include <TMRpcm.h>
//s#include "I2Cdev.h"
#include <ezButton.h>
#include "FastLED.h"
#include "MPU6050.h"

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

// ---------------------------- PINS -------------------------------
#define GYRO_PIN 2
#define BLADE_PIXEL_PIN 10
#define CONTROL_PIXEL_PIN 5
#define BLADE_BUTTON_PIN A1
#define BLADE_BUTTON_LED_PIN 6 
#define CHARACTER_BUTTON_PIN 7
#define SOUND_PIN 4
#define SPEAKER_PIN 9

// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1             // Show debug information if set to 1
#define BLADE_LEDS 60       // Number of neopixel leds for the blade
#define CONTROL_LEDS 2
#define PULSE_ALLOW 1       // blade pulsation (1 - allow, 0 - disallow)
#define PULSE_AMPL 20       // pulse amplitude
#define PULSE_DELAY 30      // delay between pulses
#define VOLUME 6           // Maximum volume (should probably not be changed if volume is controlled with potentiometer)
#define INTERRUPT_PIN 2
#define SD_ChipSelectPin 3
// ---------------------------- VARIABLES -------------------------------

CRGB pixels[BLADE_LEDS];    // Holds blade neopixels
CRGB controlPixels[CONTROL_LEDS];
TMRpcm tmrpcm;              // Define tmrpcm used for sound playback
SdFat SD;
MPU6050 accelgyro;

int pixelsPerSide = BLADE_LEDS / 2; // The neopixel strip is folded in half to light up both sides of the blade

ezButton bladeButton(BLADE_BUTTON_PIN); // Button controlling the status of the blade
ezButton characterButton(CHARACTER_BUTTON_PIN); // Button for selecting character
ezButton soundButton(SOUND_PIN);  // Button to make a random sound from the current characters sound bank

byte red, green, blue, redOffset, greenOffset, blueOffset; // RGB color information and offsets
unsigned long PULSE_timer; // Time since last pulse
int PULSEOffset;

float k = 0.2; 
bool bladeActive; 
bool firstLoop = true;

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

  Serial.println("Initializing SD card...");
  if (!SD.begin(3)) {
    //TODO: Play sound to indicate failure
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");
  
  tmrpcm.speakerPin = SPEAKER_PIN;
  tmrpcm.setVolume(VOLUME);
  tmrpcm.quality(1);

  bladeButton.setDebounceTime(50);
  characterButton.setDebounceTime(50);
  soundButton.setDebounceTime(50);
  
  setColor(0); // TODO load from eprom

  pinMode(BLADE_BUTTON_LED_PIN, OUTPUT);
  digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
  
  //FastLED.addLeds<NEOPIXEL, CONTROL_PIXEL_PIN>(controlPixels, CONTROL_LEDS);
  FastLED.addLeds<NEOPIXEL, BLADE_PIXEL_PIN>(pixels, BLADE_LEDS);
  
  
  Serial.println(F("Initializing I2C devices..."));
  // IMU initialization
  accelgyro.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (DEBUG) {
    if (accelgyro.testConnection()) Serial.println(F("MPU6050 OK"));
    else Serial.println(F("MPU6050 fail"));
  }
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
  Serial.println(freeMemory());
  delay(20);
  
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
  
  bladeButton.loop();
  characterButton.loop();
  soundButton.loop();
  
  if (!firstLoop){ // Hack that skips first loop
    if(bladeButton.isPressed()){
      if (bladeActive != true) {
         animateExtend();
          digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
       
        }
      else {
        animateRetract();
         digitalWrite(BLADE_BUTTON_LED_PIN, LOW);
       }
    }

    if(characterButton.isPressed()){
      // Character button
    }

    if(soundButton.isPressed()){   
      //Sound button
    }
  }
  firstLoop=false;
}


void animateExtend() {
  tmrpcm.play("ON.wav");
  for(int dot = 0; dot <= (BLADE_LEDS / 2 - 1); dot++) { 
      pixels[dot] = CRGB::Red;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Red;
      delay(10);
      FastLED.show();
  }  
  bladeActive = true;
}
  

void animateRetract() {
  tmrpcm.play("OFF.wav");
  for(int dot = (BLADE_LEDS / 2 - 1); dot >= 0; dot--) { 
     Serial.println(dot);
      pixels[dot] = CRGB::Black;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }
  bladeActive = false;
}



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
