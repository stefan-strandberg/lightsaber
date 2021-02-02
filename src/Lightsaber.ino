

/*
  ### SD CARD ###
  SD card attached to SPI bus as follows:
    * MOSI - pin 11
    * MISO - pin 12
    * CLK - pin 13
    * CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
    * 

  ### CONTROL BUTTONS ###
  * Led button extend and retract blade
  * 


*/

#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <TMRpcm.h>
#include "FastLED.h"

#define PIXEL_PIN        3
#define NUM_LEDS 90
CRGB pixels[NUM_LEDS];
#define DELAYVAL 500; //example

int pixelsPerSide = NUM_LEDS / 2;
int buttonState = 0;
int extended = 0;

TMRpcm tmrpcm;
const int buttonPin = 7;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(3)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(pixels, NUM_LEDS);
  pinMode(buttonPin, INPUT_PULLUP);
/*
  tmrpcm.speakerPin = 9;
  tmrpcm.setVolume(6);
  tmrpcm.quality(1);

*/ 

 
}

void loop() {
   // read the state of the pushbutton value:
  buttonState = digitalRead(7);
  /*
tmrpcm.play("ON.wav");
delay(2000);
tmrpcm.play("HUM.wav");
delay(1000);
tmrpcm.play("SK1.wav");
delay(1000);
tmrpcm.play("SK2.wav");
delay(1000);
tmrpcm.play("SK3.wav");
delay(1000);
*/

  /*
Serial.print(buttonState);
Serial.print(" - ");
Serial.println(extended);*/

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    if (extended == 0) {
      //animateExtend();
      Serial.println("Extending!");
      extended = 1;
    }
  } else {
    // turn LED off:
    if (extended == 1) {
   Serial.println("Retracting!");
   //animateRetract();
    extended = 0;
    }
  }
  
}



void animateExtend() {
 // pixels[0] = CRGB::Red; FastLED.show();

  for(int dot = 0; dot <= (NUM_LEDS / 2 - 1); dot++) { 

      pixels[dot] = CRGB::Red;
      pixels[NUM_LEDS - 1 - dot] = CRGB::Red;
      Serial.println(dot);
      // clear this led for the next time around the loop
      //leds[dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }

}
  

void animateRetract() {
  for(int dot = (NUM_LEDS / 2 - 1); dot >= 0; dot--) { 
     Serial.println(dot);
      pixels[dot] = CRGB::Black;
      pixels[NUM_LEDS - 1 - dot] = CRGB::Black;
      
      // clear this led for the next time around the loop
      //leds[dot] = CRGB::Black;
      delay(10);
      FastLED.show();
  }
}
