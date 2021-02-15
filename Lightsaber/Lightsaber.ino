

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
//#include <SD.h>
#include <SdFat.h>
SdFat SD;
#include <TMRpcm.h>
#include "I2Cdev.h"
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
// ---------------------------- VARIABLES -------------------------------
#define SD_ChipSelectPin 3
CRGB pixels[BLADE_LEDS];    // Holds blade neopixels
CRGB controlPixels[CONTROL_LEDS];
TMRpcm tmrpcm;              // Define tmrpcm used for sound playback
MPU6050 accelgyro;

int pixelsPerSide = BLADE_LEDS / 2; // The neopixel strip is folded in half to light up both sides of the blade

/*
Button bladeButton(BLADE_BUTTON_PIN); // Button controlling the status of the blade
Button characterButton(CHARACTER_BUTTON_PIN); // Button for selecting character
Button soundButton(SOUND_PIN);  // Button to make a random sound from the current characters sound bank
*/
ezButton  bladeButton(BLADE_BUTTON_PIN);
byte red, green, blue, redOffset, greenOffset, blueOffset; // RGB color information and offsets
unsigned long PULSE_timer; // Time since last pulse
int PULSEOffset;

float k = 0.2; 
bool bladeActive; 
bool firstLoop = true;



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
/*
  bladeButton.begin();
  characterButton.begin();
  soundButton.begin();
*/
bladeButton.setDebounceTime(50);
  setColor(0);

  pinMode(BLADE_BUTTON_LED_PIN, OUTPUT);
  digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
//FastLED.addLeds<NEOPIXEL, CONTROL_PIXEL_PIN>(controlPixels, CONTROL_LEDS);
Serial.print(F("Free memory start:"));
Serial.println(freeMemory());
 FastLED.addLeds<NEOPIXEL, BLADE_PIXEL_PIN>(pixels, BLADE_LEDS);
 Serial.print(F("Free memory after:"));
Serial.println(freeMemory());
//controlPixels[0] = CRGB::White; FastLED.show();
digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
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


Serial.print(F("Free memory start:"));
Serial.println(freeMemory());

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
  
 // bladePulse();
  //getFreq();
  checkInput();
  //tick();
  //strike();
  //swing();
 // tmrpcm.play("ON.wav");
   // controlPixels[0] = CRGB::White; FastLED.show(); delay(30);
 // controlPixels[0] = CRGB::Black; FastLED.show(); delay(30);
  //delay(20);
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
  if (!firstLoop){
  if(bladeButton.isPressed()){
          if (bladeActive != true) {
       animateExtend();
        //serialPrint(F("Blade Extend"));
        digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
     
      }
      else if(bladeActive == true) {
    animateRetract();
        //serialPrint(F("Blade Retracted"));
        digitalWrite(BLADE_BUTTON_LED_PIN, LOW);
     
       
      }
  }
  
  }
  firstLoop=false;
  /*
  if (bladeButton.toggled()) {
     if (bladeButton.read() == Button::PRESSED) {
      if (bladeActive != true) {
       // animateExtend();
        serialPrint("Blade Extend");
        digitalWrite(BLADE_BUTTON_LED_PIN, HIGH);
        
      }
      else {
        animateRetract();
        serialPrint("Blade Retract");
        digitalWrite(BLADE_BUTTON_LED_PIN, LOW);
      }
    } 
  }
  if (characterButton.toggled()) {
     if (characterButton.read() == Button::PRESSED) {
        serialPrint("Character changed");
        
      }
    } 
  if (soundButton.toggled()) {
     if (soundButton.read() == Button::PRESSED) {
        serialPrint("Play character sound");
        
      }
    } 
    delay(20);
    */
}


void animateExtend() {
 // pixels[0] = CRGB::Red; FastLED.show();
 tmrpcm.play("ON.wav");
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
   tmrpcm.play("OFF.wav");
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
