
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
#define FLASH_DELAY 200      // flash time while hit
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
byte currentCharacter;
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

const char vader1[] PROGMEM = "vader-as-you-wish.wav";
const char vader2[] PROGMEM = "vader-bidding.wav";
const char vader3[] PROGMEM = "vader-breath.wav";
const char vader4[] PROGMEM = "vader-fail-me.wav";
const char vader5[] PROGMEM = "vader-father.wav";
const char vader6[] PROGMEM = "vader-force.wav";
const char vader7[] PROGMEM = "vader-have-you.wav";
const char vader8[] PROGMEM = "vader-lack-of.wav";
const char vader9[] PROGMEM = "vader-yes-master.wav";

const char* const vader_sound[] PROGMEM  = {
  vader1, vader2, vader3, vader4, vader5, vader6, vader7, vader8, vader9
};

const char palpatine1[] PROGMEM = "palpatine-force-is-strong.wav";
const char palpatine2[] PROGMEM = "palpatine-no.wav";
const char palpatine3[] PROGMEM = "palpatine-power-of-the-darkside.wav";
const char palpatine4[] PROGMEM = "palpatine-secret.wav";
const char palpatine5[] PROGMEM = "palpatine-so-be-it.wav";
const char palpatine6[] PROGMEM = "palpatine-threat.wav";
const char palpatine7[] PROGMEM = "palpatine-kill-him.wav";

const char* const palpatine_sound[] PROGMEM  = {
  palpatine1, palpatine2, palpatine3, palpatine4, palpatine5, palpatine6, palpatine7
};

const char ren1[] PROGMEM = "ren-nothing-will-stand.wav";
const char ren2[] PROGMEM = "ren-show-you.wav";

const char* const ren_sound[] PROGMEM  = {
  ren1, ren2
};

const char kenobi1[] PROGMEM = "kenobi-force-flowing.wav";
const char kenobi2[] PROGMEM = "kenobi-great-disturbance.wav";
const char kenobi3[] PROGMEM = "kenobi-more-powerful.wav";
const char kenobi4[] PROGMEM = "kenobi-the-Force-will-be-with-you.wav";
const char kenobi5[] PROGMEM = "kenobi-ways-of-the-force.wav";

const char* const kenobi_sound[] PROGMEM  = {
  kenobi1, kenobi2, kenobi3, kenobi4, kenobi5
};

const char skywalker1[] PROGMEM = "skywalker-bad-feeling.wav";
const char skywalker2[] PROGMEM = "skywalker-become-yedi.wav";
const char skywalker3[] PROGMEM = "skywalker-call-me.wav";
const char skywalker4[] PROGMEM = "skywalker-darkside.wav";
const char skywalker5[] PROGMEM = "skywalker-junk.wav";

const char* const skywalker_sound[] PROGMEM  = {
  skywalker1, skywalker2, skywalker3, skywalker4, skywalker5
};

const char yoda1[] PROGMEM = "yoda-become.wav";
const char yoda2[] PROGMEM = "yoda-control.wav";
const char yoda3[] PROGMEM = "yoda-fear.wav";
const char yoda4[] PROGMEM = "yoda-no.wav";
const char yoda5[] PROGMEM = "yoda-strong.wav";
const char yoda6[] PROGMEM = "yoda-use-the-force.wav";

const char* const yoda_sound[] PROGMEM  = {
  yoda1, yoda2, yoda3, yoda4, yoda5
};

char* character_sound_bank[10];
byte totalCharacterSounds;

char BUFFER[10];

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
  tmrpcm.quality(1);

  bladeButton.setDebounceTime(50);
  characterButton.setDebounceTime(50);
  soundButton.setDebounceTime(50);
  currentCharacter = 0;
  setCharacter(currentCharacter); // TODO load from eprom

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

void loop() {
  
  bladePulse();
  humTick();
  getFreq();
  checkInput();
  swingTick();
  strikeTick();
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
    // Hum sound mode
    if (((millis() - humTimer) > 9000) && bladeActive) {
      Serial.println(F("HUMMMM!"));
      //tmrpcm.play("hum2.wav");
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
      currentCharacter++;
      if(currentCharacter < 6) {
        currentCharacter = 0;
      }
      setCharacter(currentCharacter);
    }

    if(soundButton.isPressed()){   
      playCharacterSound();
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
  //Serial.println(freeMemory());
  if (GYR > 80 && (millis() - swing_timeout > 100)) {
    swing_timeout = millis();
    if (((millis() - swing_timer) > SWING_TIMEOUT) && swing_flag && !strike_flag) {
      if (GYR >= SWING_THR) {    
        Serial.println(F("SWING1"));
        nowNumber = random(5);          
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings[nowNumber])));
        tmrpcm.play(BUFFER);
        humTimer = millis() - 9000 + swing_time[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
         
      }
      if ((GYR > SWING_L_THR) && (GYR < SWING_THR)) {
        
        nowNumber = random(4);          
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings_L[nowNumber])));
        Serial.println(F("SWING2"));
        Serial.println(nowNumber);
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
    Serial.println(F("STRIKE1"));
    //if (!HUMmode) noToneAC();
    nowNumber = random(7);
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
    Serial.println(F("STRIKE1"));
    //if (!HUMmode) noToneAC();
    nowNumber = random(7);
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


void setCharacter(byte charId){

  switch (charId) {
    case 0: // Darth Vader
      red = 255;
      green = 0;
      blue = 0;
      loadSoundBank(vader_sound);
      break;
    case 1: // Kylo Ren
      red = 255;
      green = 0;
      blue = 0;
      loadSoundBank(ren_sound);
      break;
     case 2: // Darth Sidious
      red = 255;
      green = 0;
      blue = 0;
      loadSoundBank(palpatine_sound);
      break;
     case 3: // Luke Skywalker
      red = 0;
      green = 0;
      blue = 255;
      loadSoundBank(skywalker_sound);
      break;
     case 4: // Yoda
      red = 0;
      green = 255;
      blue = 0;
      loadSoundBank(yoda_sound);
      break;
     case 5: // Obi Wan Kenobi
      red = 0;
      green = 0;
      blue = 255;
      loadSoundBank(kenobi_sound);
      break;
     case 6: // Mace Windu
      red = 125;
      green = 0;
      blue = 125;
      loadSoundBank(kenobi_sound);
      break;   
  }
}

void playCharacterSound() {
  nowNumber = random(totalCharacterSounds);
  strcpy_P(BUFFER, (char*)pgm_read_word(&(character_sound_bank[nowNumber])));
  tmrpcm.play(BUFFER);
}

void loadSoundBank(const char* const* sounds){
  totalCharacterSounds = 0;
  for (byte i = 0; i < sizeof(sounds) - 1; i++) {
          character_sound_bank[i] = sounds[i];
          totalCharacterSounds++;
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
