
// Declare FastLED
#include <FastLED.h>

#define NUM_LEDS 45

#define DATA_PIN 3

CRGB leds[NUM_LEDS];

unsigned int Effekt = 0;

bool buttonState1 = 0;
bool buttonState2 = 0;
bool buttonState3 = 1;
bool buttonState4 = 0;

//Define indicator
float Indicatorsize = 0.33;

//Define Reverse Light
int endspace = 3;   //Distance from reverslights to end of strip
int revlightsize = 5;    //Size of Reverslight on each side

void setup() {

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);

  pinMode(3, OUTPUT);
  pinMode(4, INPUT);  
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  Serial.begin(115200);
}

void loop() {

  ReadButtons();
  
  switch(Effekt) {
    case 0:   //Normal Light

      idle();
      break;

    case 1:   //Right indicator

      Indicator(0);
      break;

    case 2:   //Left indicator
    
      Indicator(1);
      break;

    case 3:   //Brakelight

      BrakeLight();      
      break;

    case 4:   //Reverse Light

      ReverseLight();
      break;
      
  }
  
}

void ReadButtons() {
  
  buttonState1 = digitalRead(4);
  buttonState2 = digitalRead(5);
  buttonState3 = digitalRead(6);
  buttonState4 = digitalRead(7);

  if(buttonState1 == HIGH) {
    Effekt = 1;
    Serial.println("right");
  }  

  if(buttonState2 == HIGH) {
    Effekt = 2;
    Serial.println("left");
  }

  if(buttonState3 == HIGH) {
    Effekt = 3;
    Serial.println("Brake");
  }  

  if(buttonState4 == HIGH) {
    Effekt = 4; 
    Serial.println("Reverse");
  }

  if(buttonState1 == LOW && buttonState2 == LOW && buttonState3 == LOW && buttonState4 == LOW) {
    Effekt = 0;
  }  

}

void idle() {

  for(int i = 0;i <= NUM_LEDS;i++) {
    leds[i].setRGB(64,0,0);
   }
   
   FastLED.show();
   
}

void Indicator(bool dir) {    //dir = 1 for left, dir = 0 for right
  float size = NUM_LEDS * Indicatorsize;

  if(dir == 1) {
     for(int i = size;i >= 0;i--) {
        leds[i].setRGB(255,50,0); 
        FastLED.show();
        delay(10);     
     }

     delay(250);
     FastLED.delay(250);

     for(int i = size;i >= 0;i--) {
       leds[i].setRGB(64,0,0);               
     }
     FastLED.show();
     delay(500);

  }

  if(dir == 0) {
    for(int i = NUM_LEDS - size;i <= NUM_LEDS;i++) {
      leds[i].setRGB(255,50,0);
      FastLED.show();
      delay(10);
    }    

    delay(250);      

    for(int i = NUM_LEDS - size;i <= NUM_LEDS;i++) {
      leds[i].setRGB(64,0,0);
    }
    
    FastLED.show();
    delay(500);

  }
}

void BrakeLight() {

  for(int i = 0;i <= NUM_LEDS;i++) {
    leds[i].setRGB(255,0,0);
  }
   FastLED.show();

}

void ReverseLight() {

  for(int i = endspace;i < endspace + revlightsize;i++) {
    leds[i].setRGB(255,255,255);
  }

  for(int i = NUM_LEDS - endspace - 1;i > NUM_LEDS - endspace - revlightsize - 1;i--) {
    leds[i].setRGB(255,255,255);
  }

  FastLED.show();

}

bool millisdelay(int millisdelay) {   //millisdelay: delay in milliseconds

  unsigned long CurrentTime = millis();
  unsigned long previousTime;

  if(currentTime - previoustime >= millisdelay) {
    previousTime = millis();
    return 1;
  }    

  return 0;

}
