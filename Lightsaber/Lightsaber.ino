
/*
 TODO:
 * Hum sound effect when blade is extended
 * Swing sound when swinging
 * Hit sound when hitting
 * Light effect when hitting
 * Play sound when sound button is pressed
 *  - Gather sound effects
 *  Character selection
 *   - Unique sounds
 *   - Blade color
 */

#include <SdFat.h>
#include <TMRpcm.h>
//s#include "I2Cdev.h"
#include <ezButton.h>
#include "FastLED.h"
#include "MPU6050.h"
#include <toneAC.h> 

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
#define VOLUME 5           // Maximum volume (should probably not be changed if volume is controlled with potentiometer)
#define INTERRUPT_PIN 2
#define SD_ChipSelectPin 3

#define SWING_TIMEOUT 500   // timeout between swings
#define SWING_L_THR 150     // swing angle speed threshold
#define SWING_THR 300       // fast swing angle speed threshold
#define STRIKE_THR 150      // hit acceleration threshold
#define STRIKE_S_THR 320    // hard hit acceleration threshold
#define FLASH_DELAY 80      // flash time while hit
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
unsigned long bzzTimer;
int PULSEOffset;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int gyroX, gyroY, gyroZ, accelX, accelY, accelZ, freq, freq_f = 20;
unsigned long ACC, GYR, COMPL;
unsigned long swing_timer, swing_timeout;
float k = 0.2; 
bool bladeActive; 
bool firstLoop = true;
boolean swing_flag, swing_allow, strike_flag;
unsigned long humTimer = -9000, mpuTimer;
byte nowNumber;
// --------------------------------- SOUNDS ----------------------------------

const char* humSound = "HUM.wav";
const char* onSound = "ON.wav";
const char* offSound = "OFF.wav";
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

  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(3)) {
    //TODO: Play sound to indicate failure
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("SD card initialized."));
  
  tmrpcm.speakerPin = SPEAKER_PIN;
  tmrpcm.setVolume(VOLUME);
  tmrpcm.quality(0);

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
  
 // bladePulse();
  humTick();
  //getFreq();
  checkInput();
  //swingTick();
  //strike();
  //swing();
 // Serial.println(freeMemory());
  //delay(20);
  
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

void humTick() {
    Serial.println(millis() - humTimer);
    // Hum sound mode
    if (((millis() - humTimer) > 9000) && bladeActive) {
      Serial.println(F("HUMMMM!"));
      tmrpcm.play("hum2.wav");
      humTimer = millis();
      swing_flag = 1;
      strike_flag = 0;
    }
   
/*
    // Generate hum
    long delta = millis() - bzzTimer;
    if ((delta > 3) && bladeActive ) {
      if (strike_flag) {
        tmrpcm.disable();
        strike_flag = 0;
      }
      toneAC(freq_f);
      bzzTimer = millis();
    } */
}

void setPixel(short Pixel, byte red, byte green, byte blue) {
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
              tmrpcm.play("HUM.wav");
        delay(200);
    }

    if(soundButton.isPressed()){   
      tmrpcm.play("hum2.wav");
      //Sound button
    }
  }
  firstLoop=false;
}


void animateExtend() {
  tmrpcm.play(onSound);
  for(int dot = 0; dot <= (BLADE_LEDS / 2 - 1); dot++) { 
      pixels[dot] = CRGB::Red;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Red;
      delay(10);
      FastLED.show();
  }  
  bladeActive = true;
}
  

void animateRetract() {
  tmrpcm.play(offSound);
  for(int dot = (BLADE_LEDS / 2 - 1); dot >= 0; dot--) { 
      pixels[dot] = CRGB::Black;
      pixels[BLADE_LEDS - 1 - dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }
  bladeActive = false;
}

void getFreq() {
  // 100% borrowed! Credits to MadGyver https://github.com/AlexGyver/EnglishProjects/blob/master/GyverSaber/GyverSaber/GyverSaber.ino
  if (bladeActive) {
    if (millis() - mpuTimer > 500) {                            
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       

      // find absolute and divide on 100
      gyroX = abs(gx / 100);
      gyroY = abs(gy / 100);
      gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      // vector sum
      ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
      ACC = sqrt(ACC);
      GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
      GYR = sqrt((long)GYR);
      COMPL = ACC + GYR;
      /*
         Serial.print("$");
         Serial.print(gyroX);
         Serial.print(" ");
         Serial.print(gyroY);
         Serial.print(" ");
         Serial.print(gyroZ);
         Serial.println(";");
      */
      freq = (long)COMPL * COMPL / 1500;                        // parabolic tone change
      freq = constrain(freq, 18, 300);                          
      freq_f = freq * k + freq_f * (1 - k);                     // smooth filter
      mpuTimer = micros();                                     
    }
  }
}

void swingTick() {
  if (GYR > 80 && (millis() - swing_timeout > 100)) {
    swing_timeout = millis();
    if (((millis() - swing_timer) > SWING_TIMEOUT) && swing_flag && !strike_flag) {
      if (GYR >= SWING_THR) {      
        nowNumber = random(5);          
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings[nowNumber])));
        tmrpcm.play(BUFFER);
        humTimer = millis() - 9000 + swing_time[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
      if ((GYR > SWING_L_THR) && (GYR < SWING_THR)) {
        nowNumber = random(5);            
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings_L[nowNumber])));
        tmrpcm.play(BUFFER);              
        humTimer = millis() - 9000 + swing_time_L[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
    }
  }
}

void strikeTick() {
  if ((ACC > STRIKE_THR) && (ACC < STRIKE_S_THR)) {
    //if (!HUMmode) noToneAC();
    nowNumber = random(8);
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes_short[nowNumber])));
    tmrpcm.play(BUFFER);
    hit_flash();
//    if (!HUMmode)
//      bzzTimer = millis() + strike_s_time[nowNumber] - FLASH_DELAY;
//    else
      humTimer = millis() - 9000 + strike_s_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
  if (ACC >= STRIKE_S_THR) {
    //if (!HUMmode) noToneAC();
    nowNumber = random(8);
    // читаем название трека из PROGMEM
    strcpy_P(BUFFER, (char*)pgm_read_word(&(strikes[nowNumber])));
    tmrpcm.play(BUFFER);
    hit_flash();
    //if (!HUMmode)
    //  bzzTimer = millis() + strike_time[nowNumber] - FLASH_DELAY;
    //else
      humTimer = millis() - 9000 + strike_time[nowNumber] - FLASH_DELAY;
    strike_flag = 1;
  }
}

void hit_flash() {
  setAllBladePixels(255, 255, 255);            
  delay(FLASH_DELAY);                
  setAllBladePixels(red, blue, green);        
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
