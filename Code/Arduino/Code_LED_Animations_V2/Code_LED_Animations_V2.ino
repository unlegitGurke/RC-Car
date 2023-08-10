  //Create TaskHandles

  TaskHandle_t Task1;
  TaskHandle_t Task2;
 
  ////////// C O R E 2 //////////

  //Lighting WS2812B using FastLED

  // Declare FastLED
  #include <FastLED.h>

  #define NUM_LEDS_BACK 45    //Declare Back LEDStrip
  #define LED_PIN_BACK 23

  #define NUM_LEDS_FRONT 34   //Declare Front LED-Strip
  #define LED_PIN_FRONT 0

  CRGB ledsback[NUM_LEDS_BACK];
  CRGB ledsfront[NUM_LEDS_FRONT];

  unsigned int Effekt = 0;

  //Define LED Groups
  
  const int fr1length = 7;    //Define Length of each front LED Section
  const int fr2length = 6;
  const int fr3length = 4;
  const int fl1length = 7;
  const int fl2length = 6;
  const int fl3length = 4;
  
  int ledsfr1[fr1length] = {0,1,2,3,4,5,6};             //Group Front LEDs to match shape they're arragned in:   - - - - - - -  fr1       fl1 - - - - - - -
  int ledsfr2[fr2length] = {12,11,10,9,8,7};            //                                                         - - - - - -  fr2       fl2 - - - - - -
  int ledsfr3[fr3length] = {13,14,15,16};               //                                                             - - - -  fr3       fl3 - - - -
  int ledsfl1[fl1length] = {17,18,19,20,21,22,23};
  int ledsfl2[fl2length] = {29,28,27,26,25,24};
  int ledsfl3[fl3length] = {30,31,32,33};

  //Define Buttons

  const int ButtonPins[6] = {13, 12, 14, 27, 15, 16};   //Pin of Buttons for Control

  bool IndicatorRightState = 0;   //Variables for button presses
  bool IndicatorLeftState = 0;
  bool BrakeLightState = 0;
  bool ReverseLightState = 0;
  bool HazardState = 0;

  //Define indicator
  float IndicatorSize = 0.20;   //Size of each LED-Strip Segment which has an Indicator

  //Define Reverse Light
  int endspace = 0;   //Distance from reverslights to end of strip
  int revlightsize = NUM_LEDS_BACK * IndicatorSize;    //Size of Reverslight on each side
  
  //Define FadeToColor
  int FadeToColorsteps = 0;
  int r1, r2, b1, b2, g1, g2;
  unsigned int d1, d2, d3;
  

  //Define Colors
  const unsigned long idlecolback = 0x400000;    //Hex Codes of each color
  const unsigned long brakecol = 0xff0000;
  const unsigned long indicatorcol = 0xff4000;
  const unsigned long reversecol = 0xffffff;
  const unsigned long frontcol = 0xffffff;
  const unsigned long frontcoldim = 0x202020;
  const unsigned long blackcol = 0x000000;
  const unsigned long idlecolfront = 0x202020;
  unsigned long EffektColor1;
  unsigned long EffektColor2;

  //Non-Blocking Delay Variables
  unsigned long currentMillisLED = 0;
  unsigned long previousMillisLED = 0;
  
  unsigned long previousMillisFadeToColor = 0;
  
  unsigned long previousMillisStartup[3] = {0, 0, 0};
  
  unsigned long previousMillisIndicator[6] = {0, 0, 0, 0, 0, 0,};

  int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
  int IndicatorOnTime = 350;  //Time the whole indicator stays on
  int IndicatorOffTime = 500; //Time between eacht indicator animation

  int StartupAnimTime = 25; //Time between each LED activatiung at startup
  int StartupFadeTime = 1000; //Time for Fade at the end of startup
  
  //State Checks
  bool IsStartup = 0;
  int StartupState[6] = {10, 0, 0, 0, 0, 0};
  int FadeToColorState = 0;
  int IndicatorState[6] = {10, 0, 0, 10, 0, 0};
  bool IndicatorInProgress[2] = {0, 0};


void setup() {

  Serial.begin(115200);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1setup,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2setup,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500);

}

void loop() {

}

void Task1setup( void * pvParameters ) {
  
  for(;;){
    Task1loop();   
  }   
  
}

void Task1loop() {
  delay(1000);
}

void Task2setup( void * pvParameters ) {
  
  FastLED.addLeds<WS2812B, LED_PIN_BACK, GRB>(ledsback, NUM_LEDS_BACK);  //Initlialize Back LEDStrip
  FastLED.addLeds<WS2812B, LED_PIN_FRONT, GRB>(ledsfront, NUM_LEDS_FRONT);  //Initialize Front LEDStrip  
    
  pinMode(LED_PIN_BACK, OUTPUT);              //Declare Pins Input/Output DEBUG Only
  pinMode(LED_PIN_FRONT, OUTPUT);
  pinMode(ButtonPins[0], INPUT);
  pinMode(ButtonPins[1], INPUT);
  pinMode(ButtonPins[2], INPUT);
  pinMode(ButtonPins[3], INPUT);
  pinMode(ButtonPins[4], INPUT);
  pinMode(ButtonPins[5], INPUT);  

  fill_solid(ledsback, NUM_LEDS_BACK, CRGB::Black);    //Turn off all LEDs at startup
  fill_solid(ledsfront, NUM_LEDS_FRONT, CRGB::Black);
  FastLED.show();

  IsStartup = 1;    //Run Startup Animation

  for(;;){
    Task2loop();
  }
    
}

void Task2loop() {
  
  currentMillisLED = millis();
  
  if(IsStartup == 1) {
    startup();
  }

  else {
    
    ReadButtons();
    idle();
    
    if(IndicatorRightState == true) {
      Indicator(0);
      IndicatorInProgress[0] = 1;
    }
    
    if(IndicatorInProgress[0] == true && IndicatorInProgress[1] == false) {
      Indicator(0);
    }
    
    if(IndicatorLeftState == true) {
      Indicator(1);
      IndicatorInProgress[1] = 1;
    }
    
    if(IndicatorInProgress[0] == false && IndicatorInProgress[1] == true) {
      Indicator(1);
    }
    
    if(HazardState == true) {
      Indicator(2);
      IndicatorInProgress[0] = 1;
      IndicatorInProgress[1] = 1;
    }
    
    if(IndicatorInProgress[0] == true && IndicatorInProgress[1] == true) {
      Indicator(2);
    }
    
    if(BrakeLightState == true) {
      BrakeLight();
    }
    
    if(ReverseLightState == true) {
      ReverseLight();
    }
    
  }

  delay(1);

}

void ReadButtons() { //Buttons for testing only
  
  IndicatorRightState = digitalRead(ButtonPins[0]);
  IndicatorLeftState = digitalRead(ButtonPins[1]);
  BrakeLightState = digitalRead(ButtonPins[2]);
  ReverseLightState = digitalRead(ButtonPins[3]);
  HazardState = digitalRead(ButtonPins[4]);
  
  //Serial.print(IndicatorRightState);
  //Serial.print(IndicatorLeftState);
  //Serial.print(BrakeLightState);
  //Serial.print(ReverseLightState);
  //Serial.println(HazardState);
  
}

void startup() {

  switch(StartupState[0]) { //Controls State of Animation sequnce
   
    case 10:
    
      StartupState[1] = NUM_LEDS_BACK / 2;
      StartupState[2] = NUM_LEDS_BACK / 2;
      StartupState[3] = 0;
      StartupState[4] = 0;
      StartupState[5] = NUM_LEDS_BACK / 2;
          
      StartupState[0]++;
      
    break;
    
    case 11:
      
      if(StartupState[1] <= NUM_LEDS_BACK && currentMillisLED - previousMillisStartup[0] >= StartupAnimTime) {
         
        ledsback[StartupState[1]] = idlecolback;
        ledsback[StartupState[2]] = idlecolback;
        SwitchFrontLedColor(StartupState[3],frontcoldim,2);   

        if(StartupState[1] - (NUM_LEDS_BACK / 2) >= 4) {
          
          ledsback[StartupState[1] - 4] = blackcol;
          ledsback[StartupState[2] + 4] = blackcol;
          
        } 
        
        if(StartupState[3] >= 1) {
          
          ledsfront[ledsfr1[constrain(StartupState[3],0,fr1length - 1) - 1]] = blackcol;
          ledsfront[ledsfr2[constrain(StartupState[3],0,fr2length - 1) - 1]] = blackcol;
          ledsfront[ledsfr3[constrain(StartupState[3],0,fr3length - 1) - 1]] = blackcol;
          ledsfront[ledsfl1[constrain(StartupState[3],0,fl1length - 1) - 1]] = blackcol;
          ledsfront[ledsfl2[constrain(StartupState[3],0,fl2length - 1) - 1]] = blackcol;
          ledsfront[ledsfl3[constrain(StartupState[3],0,fl3length - 1) - 1]] = blackcol;
          
        } 
          
        FastLED.show();

        StartupState[2]--;
        StartupState[3]++;    
        StartupState[1]++;
        previousMillisStartup[0] = currentMillisLED;
        
      }
      
      if(StartupState[1] >= NUM_LEDS_BACK) {
        
        StartupState[0]++; 
        
      }
       

    break;
    
    case 12:
    
      StartupState[2] = NUM_LEDS_BACK;
      StartupState[3] = fr1length; 
      StartupState[0]++;
    
    break;
    
    case 13:

      if(StartupState[4] <= NUM_LEDS_BACK / 2 && currentMillisLED - previousMillisStartup[1] >= StartupAnimTime) {
        ledsback[StartupState[4]] = idlecolback;
        ledsback[StartupState[2]] = idlecolback;
        SwitchFrontLedColor(StartupState[3],frontcoldim,2);

        FastLED.show();
        
        StartupState[2]--;
        StartupState[3]--;
        StartupState[4]++;
        previousMillisStartup[1] = currentMillisLED;   
      } 
    
      if(StartupState[4] >= NUM_LEDS_BACK / 2) {
        
         StartupState[0]++;
        
      }    
     

    break;
    
    case 14:
    
      StartupState[2] = NUM_LEDS_BACK / 2;
      StartupState[3] = 0;
    
      StartupState[0]++;
      
    break;
    
    case 15:
      
      if(StartupState[5] <= NUM_LEDS_BACK && currentMillisLED - previousMillisStartup[2] >= StartupAnimTime) {
        ledsback[StartupState[5]] = brakecol; 
        ledsback[StartupState[2]] = brakecol;
        SwitchFrontLedColor(StartupState[3],frontcol,2);

        FastLED.show();
        
        StartupState[2]--;
        StartupState[3]++;
        StartupState[5]++;
        previousMillisStartup[2] = currentMillisLED;
      
      }
      
      if(StartupState[5] >= NUM_LEDS_BACK) {
        
        StartupState[0]++;
        
      }     

    break;
    
    case 16:
      
      if(FadeToColor(brakecol, idlecolback, StartupFadeTime, 0, NUM_LEDS_BACK) == true) {
        
        StartupState[0]++;
        
      }
    
    break;
    
    case 17:
    
      IsStartup = 0;
      
      StartupState[0] = 10;
    
    break;
    
  }
 
}

void idle() {
  
   //If BrakeLight isnt supposed to be turned on, turn it off
   if(BrakeLightState == false) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = idlecolback;              
    }      
   }
    
    //Checks if Left Indicator is running, if not lights up left side of Strip
    if(IndicatorLeftState == false && ReverseLightState == false && HazardState == false) {
      for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
        ledsback[i] = idlecolback;
      }   
    }
    
    //Checks if Right Indicator is running, if not lights up right side of the Strip
    if(IndicatorRightState == false && ReverseLightState == false && HazardState == false) {
      for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = idlecolback;
      }
    }
    
    //If indicators arent turned on front lights idle
    if(IndicatorRightState == false && IndicatorLeftState == false && HazardState == false) {
      for(int i = 0;i <= NUM_LEDS_FRONT;i++) {    
        SwitchFrontLedColor(i,frontcoldim,2);
      }
    }
    
    FastLED.show();
  
}

void BrakeLight() {
  
  if(BrakeLightState == true) {
    
    //Lights up Center of Strip
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = brakecol;
    }
    
    //Checks if Left Indicator is running, if not lights up left side of Strip
    if(IndicatorLeftState == false && ReverseLightState == false && HazardState == false && BrakeLightState == true) {
      for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
        ledsback[i] = brakecol;
      }
      
    }
    
    //Checks if Right Indicator is running, if not lights up right side of the Strip
    if(IndicatorRightState == false && ReverseLightState == false && HazardState == false && BrakeLightState == true) {
      for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = brakecol;
      }
    
    }
    
    FastLED.show();
    
  }
  
}

void ReverseLight() {  

  if(IndicatorRightState == false && HazardState == false && ReverseLightState == true) {  
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <=NUM_LEDS_BACK;i++) {
      ledsback[i] = reversecol;
    }
  }

  if(IndicatorLeftState == false && HazardState == false && ReverseLightState == true) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0 ;i--) {
      ledsback[i] = reversecol;
    }
  }

  FastLED.show();

}

void Indicator(int dir) {    //dir = 1 for left, dir = 0 for right

  if(dir == 1 || dir == 2) {    //Left Indicator
    
    switch(IndicatorState[0]) {
      
      case 10:
        
        IndicatorState[1] = 0;
        IndicatorState[2] = NUM_LEDS_BACK * IndicatorSize;
        
        IndicatorState[0]++;
        
      break;
      
      case 11:
      
        if(IndicatorState[2] >= 0 && currentMillisLED - previousMillisIndicator[0] >= IndicatorAnimTime) {   //Animation 
        
          ledsback[IndicatorState[2]] = indicatorcol; 
          SwitchFrontLedColor(IndicatorState[1],indicatorcol,1);
          FastLED.show();   
          
          IndicatorState[2]--;
          IndicatorState[1]++;
          previousMillisIndicator[0] = currentMillisLED; 
          
        }
        
        if(IndicatorState[2] < 0) {
          
          IndicatorState[0]++;
          previousMillisIndicator[1] = currentMillisLED;
          
        }
      
      break;
      
      case 12:

        if(currentMillisLED - previousMillisIndicator[1] >= IndicatorOnTime) {
          
          IndicatorState[0]++;
          IndicatorState[1] = 0;
          
        }
      
      break;
      
      case 13:
      
        for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0;i--) {
          
          ledsback[i] = idlecolback;
          SwitchFrontLedColor(IndicatorState[1],idlecolfront,1);
          
          IndicatorState[1]++; 
                        
        }
        
        FastLED.show();
        
        IndicatorState[0]++;
        previousMillisIndicator[2] = currentMillisLED;
      
      break;
      
      case 14:
      
        if(currentMillisLED - previousMillisIndicator[2] >= IndicatorOffTime) {
          
          IndicatorState[0] = 10;
          
          IndicatorInProgress[1] = 0;
          
        }
      
      break;
      
    }
    
  }

  if(dir == 0 || dir == 2) {    //Right Indicator 
  
    switch(IndicatorState[3]) {
      
      case 10:
      
        IndicatorState[3]++;
        IndicatorState[4] = 0;
        IndicatorState[5] = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;
      
      break;
      
      case 11:
      
        if(IndicatorState[5] <= NUM_LEDS_BACK && currentMillisLED - previousMillisIndicator[3] >= IndicatorAnimTime) {    //Animation
        
          ledsback[IndicatorState[5]] = indicatorcol;
          SwitchFrontLedColor(IndicatorState[4],indicatorcol,0);
          
          FastLED.show();
          
          IndicatorState[4]++;
          IndicatorState[5]++;

          previousMillisIndicator[3] = currentMillisLED;       
          
        } 
        
        if(IndicatorState[5] > NUM_LEDS_BACK) {
          
          IndicatorState[3]++;
          previousMillisIndicator[4] = currentMillisLED;
          
        }
      
      break;
      
      case 12:
      
        if(currentMillisLED - previousMillisIndicator[4] >= IndicatorOnTime) {
          
          IndicatorState[3]++;
          IndicatorState[4] = 0;
          
        }
      
      break;
      
      case 13:
      
        for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {
          ledsback[i] = idlecolback;
          SwitchFrontLedColor(IndicatorState[4],idlecolfront,0);
          IndicatorState[4]++;
        }
        
        FastLED.show();
        
        IndicatorState[3]++;
        previousMillisIndicator[5] = currentMillisLED;
      
      break;
      
      case 14:
        
        if(currentMillisLED - previousMillisIndicator[5] >= IndicatorOffTime) {
          
          IndicatorState[3] = 10;
          
          IndicatorInProgress[0] = 0;
          
        }
      
      break;
    }
    
  }
  
}

bool FadeToColor(unsigned long Color1, unsigned long Color2, unsigned int timeinms, int ledbegin, int ledend) {     //This function fades all the LEDs from Color1 to Color 2 in given time "timeinms" Color1 and Color2 should be HEX Values format: "0x000000"

  //Convert Color1 from HEX to RGB this is done by splitting the HEX Value in 3 Parts and converting them do decimal
  if(FadeToColorState == 0) {
    r1 = Color1 >> 16;

    g1 = (Color1 & 0x00ff00) >> 8;

    b1 = (Color1 & 0x0000ff);

    //Convert Color2 from HEX to RGB
    r2 = Color2 >> 16;

    g2 = (Color2 & 0x00ff00) >> 8;

    b2 = (Color2 & 0x0000ff);
    
    //Calsculate Differences between Color1 and Color2 RGB Values
    
    d1 = abs(r1 -r2);
    
    d2 = abs(g1 - g2);
    
    d3 = abs(b1 -b2);
  
    //Determine how many steps it takes to Fade from Color1 to Color2, the biggest differnce of the RGB Values is taken
    
    if(d1 >= d2 || d1 >= d3) {
      FadeToColorsteps = d1;
    }
    
    else if(d2 >= d1 || d2 >= d3) {
      FadeToColorsteps = d2;
    }
    
    else if(d3 >= d1 || d3 >= d2) {
      FadeToColorsteps = d3;
    }
  }
  
  //Fade the colors
  
  if(FadeToColorState <= FadeToColorsteps /*&& FadeToColorsteps > 0*/ ) {    
    if(currentMillisLED - previousMillisFadeToColor >= (timeinms / FadeToColorsteps)) {
      //Checks if current of r1 value is under, over or equals the final value r2 and adjusts it one step towards r2
      if(r1 > r2) {
        r1--;
      }
      
      else if(r1 == r2) {}
      
      else if(r1 < r2) {
        r1++;
      }
      
      //Checks if current of g1 value is under, over or equals the final value g2 and adjusts it one step towards g2
      if(g1 > g2) {
        g1--;
      }
      
      else if(g1 == g2) {}
      
      else if(g1 < g2) {
        g1++;
      }
      
      //Checks if current of b1 value is under, over or equals the final value b2 and adjusts it one step towards b2
      if(b1 > b2) {
        b1--;
      }
      
      else if(b1 == b2) {}
      
      else if(b1 < b2) {
        b1++;
      }
      
      for(int i = ledbegin; i <= ledend;i++) {   //All the LEDS are set to the current RGB Values
        ledsback[i].setRGB(r1, g1, b1);  
      }
      
      FastLED.show();
      
      FadeToColorState++;
      
      previousMillisFadeToColor = currentMillisLED;
      
      return false;
      
      //delay(timeinms / FadeToColorsteps);
      
    }
  }
  
  else {
    
    FadeToColorState = 0;
    return true;
    
  }
  
}

void SwitchFrontLedColor(int nrofled, long color, int dir) {        //dir = 1 for left, dir  0 for right, dir = 2 for both, dir = 3 for both without the 3rd row of leds

  if(dir == 2) {  //Both Sides will be affected
    
    ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    ledsfront[ledsfr3[constrain(nrofled,0,fr3length - 1)]] = color;
    ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    ledsfront[ledsfl3[constrain(nrofled,0,fl3length - 1)]] = color;
  }  

  if(dir == 0) {  //Only the right side will be affected
    
    //ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    //ledsfront[ledsfr3[constrain(nrofled,0,fr3length - 1)]] = color;
    
  }

  if(dir == 1) {  //Only the left side will be affected
    
    //ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    //ledsfront[ledsfl3[constrain(nrofled,0,fl3length - 1)]] = color;
  }
  
  if(dir == 3) {  //Both sides will be affected except the bottom row of leds
    
    ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    
  }
  
}