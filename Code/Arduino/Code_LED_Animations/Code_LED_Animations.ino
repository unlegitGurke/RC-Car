#include <FastLED.h>

#define NUM_LEDS 45

#define DATA_PIN 3

CRGB leds[NUM_LEDS];

unsigned int Effekt = 0;

bool buttonState1 = 0;
bool buttonState2 = 0;

void setup() {

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);

  pinMode(4, OUTPUT);  
  pinMode(5, OUTPUT);

  Serial.begin(115200);
}

void loop() {

  buttonState1 = digitalRead(4);
  buttonState2 = digitalRead(5);

  if(buttonState1 == HIGH) {
    Effekt = 1;
    Serial.println("rechts");
  }  

  if(buttonState2 == HIGH) {
    Effekt = 2;
    Serial.println("links");
  }

  if(buttonState1 == LOW && buttonState2 == LOW) {
    Effekt = 0;
  }
  
  switch(Effekt) {
    case 0:

      for(int i = 0;i <= 45;i++) {
        leds[i] = CRGB::Red;
      }
      FastLED.show();
      break;

    case 1:

      for(int i = 29;i <= 45;i++) {
        leds[i] = CRGB::DarkOrange;
        FastLED.show();
        delay(10);        
      }

      delay(200);

      for(int i = 29;i <= 45;i++) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      delay(500);
      break;

    case 2:
    
      for(int i = 16;i >= 0;i--) {
        leds[i] = CRGB::DarkOrange;
        FastLED.show();
        delay(10);
        
      }

      delay(200);

      for(int i = 16;i >= 0;i--) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      delay(500);
      break;

  }
  
}
