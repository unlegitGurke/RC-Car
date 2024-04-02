// Declare FastLED
#include <FastLED.h>

#define NUM_LEDS 45
#define DATA_PIN 23

CRGB leds[NUM_LEDS];

unsigned int Effekt = 0;

//Define Buttons

const int ButtonPins[6] = {13, 12, 14, 27, 15, 16};   //Pin of Buttons for Control

bool IndicatorRightState = 0;   //Variables for button presses
bool IndicatorLeftState = 0;
bool BrakeLightState = 1;
bool ReverseLightState = 0;
bool HazardState = 0;

//Define indicator
float IndicatorSize = 0.20;   //Size of each LED-Strip Segment

//Define Reverse Light
int endspace = 0;   //Distance from reverslights to end of strip
int revlightsize = NUM_LEDS * IndicatorSize;    //Size of Reverslight on each side

//Define Colors
#define idlecol 0x400000    //Hex Codes ofeach color
#define brakecol 0xff0000
#define indicatorcol 0xff4000
#define reversecol 0xffffff

//Non-Blocking Delay Variables
int currentMillis = 0;
int previousmillis = 0;

int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
int IndicatorOffTime = 500; //Time between eacht indicator animation

void setup() {
  delay(3000);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);  //Initlialize Back LEDStrip
    
  pinMode(DATA_PIN, OUTPUT);              //Declare Pins Input/Output
  pinMode(ButtonPins[0], INPUT);
  pinMode(ButtonPins[1], INPUT);
  pinMode(ButtonPins[2], INPUT);
  pinMode(ButtonPins[3], INPUT);
  pinMode(ButtonPins[4], INPUT);
  pinMode(ButtonPins[5], INPUT);  

  fill_solid(leds, NUM_LEDS, CRGB::Black);    //Turn off all LEDs at startup

  Serial.begin(115200);   //Initialize Serial Monitor, Used for Debugging
}

void loop() {

  ReadButtons();
  
  //Turns on effects based on Effect Variable, Brake Light is also called when not in use to turn it off
  switch(Effekt) {
    case 0:   //Normal Light

      idle();
      break;

    case 1:   //Right indicator

      BrakeLight();
      Indicator(0);
      break;

    case 2:   //Left indicator

      BrakeLight();
      Indicator(1);
      break;

    case 3:   //Brakelight

      BrakeLight();      
      break;

    case 4:   //Reverse Light

      BrakeLight();
      ReverseLight();
      break;
    
    case 5:   //Hazard Light

      Indicator(2);                
      break;

    case 6:  //Right Indicator + Brake Light    

      BrakeLight();
      Indicator(0);
      break;

    case 7:  //Left Indicator + Brake Light

      BrakeLight();
      Indicator(1);
      break;      

    case 8:  //Hazard Light + Brake Light

      BrakeLight();
      Indicator(2);
      break;

    case 9:   //Reverse Light + Brake Light
    
      BrakeLight();
      ReverseLight();
      break;
    
    case 10:  //Right Indicator + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(0);
      break;    

    case 11:  //Left Indicator + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(1);
      break;

    case 12:  //Hazard Lights + Reverse Light

      BrakeLight();
      Indicator(2);      
      break;

    case 13:  //Right Indicator + Brake Light + Reverse Light
      
      BrakeLight();
      ReverseLight();
      Indicator(0);
      break;

    case 14:  //Left Indicator + Brake Light + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(1);
      break;

    case 15:  //Hazard Lights + Brake Light + Reverse Light
    
      BrakeLight();
      Indicator(2);
      break;     
       
  }
  
}

void ReadButtons() {
  
  UpdateButtonState();    //Updates ButtonStates

  if(IndicatorLeftState == HIGH && HazardState == HIGH || IndicatorRightState == HIGH && HazardState == HIGH ){   //If Hazards and Indicators are on at the same time the indicators can be turned off
    IndicatorLeftState == LOW;
    IndicatorRightState == LOW;
  }  

  //Determines which effects must be displayed depending on Button presses
  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 0;
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 1;
    Serial.println("right");
  }  

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 2;
    Serial.println("left");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 3;
    Serial.println("Brake");
  }  

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 4; 
    Serial.println("Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 5; 
    Serial.println("Hazard Lights");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 6; 
    Serial.println("Right + Brake");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 7; 
    Serial.println("Left + Brake");
  }

  if(HazardState == HIGH && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 8; 
    Serial.println("Hazard Lights + Brake");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 9; 
    Serial.println("Reverse + Brake");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 10; 
    Serial.println("Right + Reverse");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 11; 
    Serial.println("Left + Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 12; 
    Serial.println("Hazard Lights + Reverse");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 13; 
    Serial.println("Right + Brake + Reverse");
  }
  
  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 14; 
    Serial.println("Left + Brake + Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 15; 
    Serial.println("Hazard Lights + Brake + Reverse");
  }
}

void UpdateButtonState() {
  //Read alle the Buttonstates
  IndicatorRightState = digitalRead(ButtonPins[0]);
  IndicatorLeftState = digitalRead(ButtonPins[1]);
  BrakeLightState = digitalRead(ButtonPins[2]);
  ReverseLightState = digitalRead(ButtonPins[3]);
  HazardState = digitalRead(ButtonPins[4]);
}

void idle() {

  for(int i = 0;i <= NUM_LEDS;i++) {    //State of LEDs when nothing is happening
    leds[i] = idlecol;
   }
   
   FastLED.show();
   
}

void Indicator(int dir) {    //dir = 1 for left, dir = 0 for right, dir = 2 for both

  if(dir == 1) {    //Left Indicator
     for(int i = NUM_LEDS * IndicatorSize;i >= 0;i--) {   //Animation 
        leds[i] = indicatorcol; 
        FastLED.show();
        //delay(IndicatorAnimTime);
        mydelay(IndicatorAnimTime);              
     }

     //delay(IndicatorOffTime / 2);
     mydelay(IndicatorOffTime / 2);
     
     for(int i = NUM_LEDS * IndicatorSize;i >= 0;i--) {
       leds[i] = idlecol;               
     }
     FastLED.show();
     //delay(IndicatorOffTime);
     mydelay(IndicatorOffTime);

  }

  if(dir == 0) {    //Right Indicator 
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {    //Animation
      leds[i] = indicatorcol;
      FastLED.show();
      //delay(IndicatorAnimTime);
      mydelay(IndicatorAnimTime);                
       
    }    

    //delay(IndicatorOffTime / 2);
    mydelay(IndicatorOffTime / 2);
        
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {
      leds[i] = idlecol;
    }
    
    FastLED.show();
    
    //delay(IndicatorOffTime);
    mydelay(IndicatorOffTime);
         
    dir = 0;

  }

  if(dir == 2) {    //Hazard Lights

    int y = NUM_LEDS * IndicatorSize;   //Variable for Right Indicator
    
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {  //i used for left, y used for right side
      leds[i] = indicatorcol;
      leds[y] = indicatorcol;
      FastLED.show();
      y--;
      //delay(10);
      mydelay(IndicatorAnimTime);
      
    }    

    //delay(250);
    mydelay(IndicatorOffTime / 2);  
        
    y = NUM_LEDS * IndicatorSize;     //Reset y
    
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {    //Turn everything Black
      leds[i] = idlecol;
      leds[y] = idlecol;
      y--;
    }
    
    FastLED.show();
    //delay(500);
    mydelay(IndicatorOffTime);

  } 

     
}

void BrakeLight() {

  UpdateButtonState();
  ReverseLight();

  //If BrakeLight isnt supposed to be turned on, turn it off
  if(BrakeLightState == LOW) {
    for(int i = NUM_LEDS * IndicatorSize;i <= NUM_LEDS - NUM_LEDS * IndicatorSize;i++) {
      leds[i] = idlecol;              
    }      
  }
  
  else {

    //Lights up Center of Strip
    for(int i = NUM_LEDS * IndicatorSize;i <= NUM_LEDS - NUM_LEDS * IndicatorSize;i++) {
      leds[i] = brakecol;
    }
  }  
  
  //Checks if Left Indicator is running, if not lights up left side of Strip
  if(IndicatorLeftState == LOW && ReverseLightState == LOW && IndicatorRightState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
    for(int i = 0; i <= NUM_LEDS * IndicatorSize;i++) {
      leds[i] = brakecol;
    }
  } 

  else if(IndicatorLeftState == HIGH || HazardState == HIGH || ReverseLightState == HIGH) {
    
  }

  else {
    for(int i = 0; i <= NUM_LEDS * IndicatorSize;i++) {
      leds[i] = idlecol;
    }    
  }

  //Checks if Right Indicator is running, if not lights up right side of the Strip
  if(IndicatorRightState == LOW && ReverseLightState == LOW && IndicatorLeftState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize; i <= NUM_LEDS;i++) {
      leds[i] = brakecol;
    }
  }   

  else if(IndicatorRightState == HIGH || HazardState == HIGH || ReverseLightState == HIGH) {
    
  }
  
  else {
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize; i <= NUM_LEDS;i++) {
      leds[i] = idlecol;
    }    
  }

  FastLED.show();
}

void ReverseLight() {  

  if(IndicatorRightState == LOW && HazardState == LOW && ReverseLightState == HIGH) {  
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <=NUM_LEDS;i++) {
      leds[i] = reversecol;
    }
  }

  if(IndicatorLeftState == LOW && HazardState == LOW && ReverseLightState == HIGH) {
    for(int i = NUM_LEDS * IndicatorSize;i >= 0 ;i--) {
      leds[i] = reversecol;
    }
  }

  FastLED.show();

}

void mydelay(int timeinms) {    //Time to wait in milliseconds

  currentMillis = previousmillis = millis();
  while(previousmillis + timeinms >= currentMillis) {
    BrakeLight();
    currentMillis = millis();
  }    
  
}

