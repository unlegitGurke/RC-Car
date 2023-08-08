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
  bool BrakeLightState = 1;
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
  int currentMillisLED = 0;
  int previousMillisLED = 0;
  
  int previousMillisFadeToColor = 0;

  int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
  int IndicatorOnTime = 250;
  int IndicatorOffTime = 500; //Time between eacht indicator animation

  int StartupAnimTime = 25; //Time between each LED activatiung at startup
  int StartupFadeTime = 1000; //Time for Fade at the end of startup
  
  //State Checks
  bool IsStartup = 0;
  int StartupState = 10;
  int FadeToColorState = 0;


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

  delay(1);

}

void startup() {

  Serial.println("Test");

  int y;
  int z;

  switch(StartupState) { //Controls State of Animation sequnce
    
    case 10:
    
      y = NUM_LEDS_BACK / 2;
      z = 0;
      
      for(int i = NUM_LEDS_BACK / 2;i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = idlecolback;
        ledsback[y] = idlecolback;
        SwitchFrontLedColor(z,frontcoldim,2);   

        if(i - (NUM_LEDS_BACK / 2) >= 4) {
          ledsback[i -4] = blackcol;
          ledsback[y + 4] = blackcol;
        } 
        
        if(z >= 1) {
          
          ledsfront[ledsfr1[constrain(z,0,fr1length - 1) - 1]] = blackcol;
          ledsfront[ledsfr2[constrain(z,0,fr2length - 1) - 1]] = blackcol;
          ledsfront[ledsfr3[constrain(z,0,fr3length - 1) - 1]] = blackcol;
          ledsfront[ledsfl1[constrain(z,0,fl1length - 1) - 1]] = blackcol;
          ledsfront[ledsfl2[constrain(z,0,fl2length - 1) - 1]] = blackcol;
          ledsfront[ledsfl3[constrain(z,0,fl3length - 1) - 1]] = blackcol;
          
        } 
          
        FastLED.show();

        y--;
        z++;    
        delay(StartupAnimTime);
      }
      
      StartupState++;  

    break;
    
    case 11:
    
      StartupState++;
      delay(1);

    break;
    
    case 12:
    
       y = NUM_LEDS_BACK;
       z = fr1length;

      for(int i = 0; i <= NUM_LEDS_BACK / 2;i++) {
        ledsback[i] = idlecolback;
        ledsback[y] = idlecolback;
        SwitchFrontLedColor(z,frontcoldim,2);

        FastLED.show();
        
        y--;
        z--;
        delay(StartupAnimTime);    
      } 
    
      StartupState++;

    break;
    
    case 13:
    
      StartupState++;
      delay(1);

    break;
    
    case 14:
    
       y = NUM_LEDS_BACK / 2;
       z = 0;

      for(int i = NUM_LEDS_BACK / 2;i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = brakecol; 
        ledsback[y] = brakecol;
        SwitchFrontLedColor(z,frontcol,2);

        FastLED.show();
        
        y--;
        z++;
        delay(StartupAnimTime);       
      }
      
      StartupState++;

    break;
    
    case 15:
    
      StartupState++;
      delay(1);
    
    break;
    
    case 16:
      
      if(FadeToColor(brakecol, idlecolback, StartupFadeTime, 0, NUM_LEDS_BACK) == true) {
        
        StartupState++;
        
      }
    
    break;
    
    case 17:
    
      IsStartup = 0;
      
      StartupState = 10;
    
    break;
    
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