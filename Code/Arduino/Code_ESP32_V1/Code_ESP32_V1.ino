//Create TaskHandles

TaskHandle_t Task1;
TaskHandle_t Task2;

////////// C O R E 1 //////////

//Declare PWM Fan Controller

const int Fan_Pin[4] = {17, 5, 18, 19};

const int PWMChannel[4] = {0, 2, 4, 6};

const int Resolution[4] = {10, 10, 10, 10};

const int Frequency[4] = {25000, 25000, 25000, 25000};

int FanSpeed[4] = {25, 50, 75, 90};

int DutyCycle[4] = {0, 0, 0, 0};


//Declare Accelerometer and Gyroscope

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

const unsigned int ReadIntervallIMU = 200; //Time between Sensor Readings in milliseconds
unsigned long previousMillisIMU = 0;
unsigned long currentMillisIMU;


//Declare Ultrasonic Sensors

#include "OctoSonar.h"

#define SONAR_ADDR 0x20
#define SONAR_INT 2
#define ACTIVE_SONARS 0xFFFF

const int ReadIntervallSonar = 200; //Time between Sensor Readingsd in millisceonds
uint32_t last_print = 0;
uint8_t SonarValues[16];

OctoSonarX2 myOcto(SONAR_ADDR, SONAR_INT);

//Declare Temperature Sensors

const float temprangemin = 218.15;    //Temperature Range of thermistor in Kelvin
const float temprangemax = 398.15;

const int NrOfSensors = 6;              //Define Number of Sensors connected to the Analog Pins of the Arduino
const int SensorPins[NrOfSensors] = {34,35,32,33,25,26}; //Define Pins of Sensors

const int TempUnit = 2;   //Choose which Unit the Temperature will be printed in, 1 for Kelvin, 2 for Celcius, 3 for Fahrenheit

const int ADCRes = 12;  //Resolution of ADC in Bits (Arduino Nano: 10, ESP32: 12)

const int ErrorLedPin = 4; //Pin for Error LED

const unsigned int ReadIntervallTemp = 200; //Time between Sensor Readings in milliseconds
unsigned long previousMillisTemp = 0;
unsigned long currentMillisTemp;

float temp[NrOfSensors + 1][3];

bool Error[NrOfSensors];
bool IsError = 0;

////////// C O R E 2 //////////

//Lighting WS2812B using FastLED

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
int currentMillisLED = 0;
int previousMillisLED = 0;

int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
int IndicatorOnTime = 250;
int IndicatorOffTime = 500; //Time between eacht indicator animation

void setup(){
  
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

void loop(){
  
}

////////// C O R E 1 //////////

void Task1setup( void * pvParameters ){    //Task1 Core 0
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  delay(3000);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  myOcto.begin(ACTIVE_SONARS);   // initialize bus, pins etc

  pinMode(ErrorLedPin, OUTPUT);

  //Fan Control

  ledcAttachPin(Fan_Pin[0], PWMChannel[0]);
  ledcAttachPin(Fan_Pin[1], PWMChannel[1]);
  ledcAttachPin(Fan_Pin[2], PWMChannel[2]);
  ledcAttachPin(Fan_Pin[3], PWMChannel[3]);

  ledcSetup(PWMChannel[0], Frequency[0], Resolution[0]);
  ledcSetup(PWMChannel[1], Frequency[1], Resolution[1]);
  ledcSetup(PWMChannel[2], Frequency[2], Resolution[2]);
  ledcSetup(PWMChannel[3], Frequency[3], Resolution[3]);

  for(;;){
    Task1loop();   
  } 
}

void Task1loop() {
  
  ReadTemp();
  ReadSonar();
  ReadIMU();
  Fan_Control();

  delay(1);
}

void ReadTemp() {

  currentMillisTemp = millis();
  if(currentMillisTemp - previousMillisTemp >= ReadIntervallTemp) {
    previousMillisTemp = currentMillisTemp;
  
    IsError = 0;
    
    for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them
      SaveTemp(SensorPins[i], i);
      Serial.print("Temp");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(temp[i][TempUnit]);
      Serial.print(" C Error: ");
      Serial.println(Error[i]);
  
      if(Error[i] == 1) {
        IsError = 1;              
      }
      
    }
  
    if(IsError == 1) {
      digitalWrite(ErrorLedPin, HIGH);
    }
    else {
      digitalWrite(ErrorLedPin, LOW);
    }
    
    Serial.println(" ");
    
  }
  
}

void SaveTemp(int Pin, int Pos) {   //Pin of Sensor and Position of Value in temp Array
  
  int tempReading = analogRead(Pin);
  double tempK = log(10000.0 * ((pow(2,ADCRes) / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit

  if(tempK > temprangemin && tempK < temprangemax) {
    temp[Pos][1] = tempK;
    temp[Pos][2] = tempC;
    temp[Pos][3] = tempF;    
    Error[Pos] = 0;
  }
  else {
    temp[Pos][1] = 0;
    temp[Pos][2] = 0;
    temp[Pos][3] = 0;    
    Error[Pos] = 1;
  }
  
}

void ReadSonar() {
  OctoSonar::doSonar();  // call every cycle, OctoSonar handles the spacing

  if (last_print + ReadIntervallSonar < millis()) {   
    last_print = millis();
    for (uint8_t i = 0; i < 16; i++) {
      SonarValues[i] = myOcto.read(i);
      Serial.print(SonarValues[i]); Serial.print("   ");
      
    }
    Serial.println();
  }  
}

void ReadIMU() {
  currentMillisIMU = millis();
  if(currentMillisIMU - previousMillisIMU >= ReadIntervallIMU) {
    previousMillisIMU = currentMillisIMU;
  
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    Serial.println("  ");

  }    
}

void Fan_Control() {

  for(int i = 0;i < 4;i++) {

    DutyCycle[i] = map(FanSpeed[i], 0, 100, 0, pow(2, Resolution[i]));

    ledcWrite(PWMChannel[i], DutyCycle[i]);

  }

}

////////// C O R E 2 //////////

void Task2setup( void * pvParameters ){    //Task2 Core 1
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

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

  startup();    //Run Startup Animation

  for(;;){
    Task2loop();
  }
}

void Task2loop() {  

  ReadButtons();
  Effect();

  
  //delay(1);
}

void startup() {
  for(int i = NUM_LEDS / 2;i <= NUM_LEDS;i++) {
    leds[i] = brakecol;

    if((NUM_LEDS / 2) - i >= 4) {
      leds[i -3] = #000000;
    }    

  } 
  
}

void Effect() {
  
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

void ReadButtons() {    //This is an extra function, as it will likely not be used in the final version of the project
  
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
    //Serial.println("right");
  }  

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 2;
    //Serial.println("left");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 3;
    //Serial.println("Brake");
  }  

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 4; 
    //Serial.println("Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == LOW && ReverseLightState == LOW) {
    Effekt = 5; 
    //Serial.println("Hazard Lights");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 6; 
    //Serial.println("Right + Brake");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 7; 
    //Serial.println("Left + Brake");
  }

  if(HazardState == HIGH && BrakeLightState == HIGH && ReverseLightState == LOW) {
    Effekt = 8; 
    //Serial.println("Hazard Lights + Brake");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 9; 
    //Serial.println("Reverse + Brake");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 10; 
    //Serial.println("Right + Reverse");
  }

  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 11; 
    //Serial.println("Left + Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == LOW && ReverseLightState == HIGH) {
    Effekt = 12; 
    //Serial.println("Hazard Lights + Reverse");
  }

  if(IndicatorRightState == HIGH && IndicatorLeftState == LOW && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 13; 
    //Serial.println("Right + Brake + Reverse");
  }
  
  if(IndicatorRightState == LOW && IndicatorLeftState == HIGH && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 14; 
    //Serial.println("Left + Brake + Reverse");
  }

  if(HazardState == HIGH && BrakeLightState == HIGH && ReverseLightState == HIGH) {
    Effekt = 15; 
    //Serial.println("Hazard Lights + Brake + Reverse");
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

     //delay(IndicatorOnTime);
     mydelay(IndicatorOnTime);
     
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

    //delay(IndicatorOnTime);
    mydelay(IndicatorOnTime);
        
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {
      leds[i] = idlecol;
    }
    
    FastLED.show();
    
    //delay(IndicatorOffTime);
    mydelay(IndicatorOffTime);

  }

  if(dir == 2) {    //Hazard Lights

    int y = NUM_LEDS * IndicatorSize;   //Variable for Right Indicator
    
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {  //i used for left, y used for right side
      leds[i] = indicatorcol;
      leds[y] = indicatorcol;
      FastLED.show();
      y--;
      //delay(IndicatorAnimTime);
      mydelay(IndicatorAnimTime);
      
    }    

    //delay(IndicatorOnTime);
    mydelay(IndicatorOnTime);  
        
    y = NUM_LEDS * IndicatorSize;     //Reset y
    
    for(int i = NUM_LEDS - NUM_LEDS * IndicatorSize;i <= NUM_LEDS;i++) {    //Turn everything Black
      leds[i] = idlecol;
      leds[y] = idlecol;
      y--;
    }
    
    FastLED.show();
    //delay(IndicatorOffTime);
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
  if(IndicatorLeftState == LOW && ReverseLightState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
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
  if(IndicatorRightState == LOW && ReverseLightState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
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

  currentMillisLED = previousMillisLED = millis();
  while(previousMillisLED + timeinms >= currentMillisLED) {
    BrakeLight();
    currentMillisLED = millis();
  }    
  
}