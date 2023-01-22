//////// S E T U P //////////

  //Create TaskHandles

  TaskHandle_t Task1;
  TaskHandle_t Task2;

  //Libraries used across everything

  #include <math.h>

  ////////// C O R E 1 //////////

  //General Values

  const int ADCRes = 12;  //Resolution of ADC in Bits (Arduino Nano: 10, ESP32: 12)
  const int ErrorCnt = 11;   //Amount of Erro Variables 
  bool Error[ErrorCnt] = {0,0,0,0,0,0,0,0,0,0,0};   //Error Codes are saved in here:  0 - 6 : Temp Sensor Error, 7 - 11 : Max Voltage too HIGH for Sensor Microcontroller can be harmed!
  bool IsError = 0;    //True if theres any Error
  bool IsErrorPanda = 0;    //True, if theres an Error on the LattePanda

  //Declare PWM Fan Controller

  const int Fan_Pin[4] = {17, 5, 18, 19};   //Pins, the PWM Pins of the fans are connected to

  const int PWMChannel[4] = {0, 2, 4, 6};   //Which PWM Channel of Microncontroller will be used

  const int Resolution[4] = {10, 10, 10, 10};   //ADC Resolution

  const int Frequency[4] = {25000, 25000, 25000, 25000};    //PWM Frequency of the Fans

  int FanSpeed[4] = {0, 0, 0, 0};   //Speed of the fans at the beginning of the program

  int DutyCycle[4] = {0, 0, 0, 0};    //Fanspeed converted to a value for the ADC

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

  const int ErrorLedPin = 4; //Pin for Error LED
  
  int TempUnit = 2;   //Choose which Unit the Temperature will be printed in, 1 for Kelvin, 2 for Celcius, 3 for Fahrenheit

  const unsigned int ReadIntervallTemp = 200; //Time between Sensor Readings in milliseconds
  unsigned long previousMillisTemp = 0;
  unsigned long currentMillisTemp;

  float temp[NrOfSensors + 1][3];

  //Declare Voltage Sensors

  const float R1[] = {100000, 100000, 100000, 100000, 100000};   //Voltage Divider Resistor Values
  const float R2[] = {4700, 4700, 4700, 4700, 4700};

  const int NOSVoltage = 5;   //Number of Sensors plugged in
  const int VoltagePin[] = {12, 13, 14, 15, 27};  //Sensor Pins
  const float LogicLevel = 3.3;   //Logic Level of the microcontroller
  const float MaxVoltage = 80;  //Max Voltage that will be measured

  float Voltage[NOSVoltage];  //Voltage Data will be saved in here
  float InputVoltage[5] = {0,0,0,0,0};

  const unsigned int ReadIntervallVoltage = 10;   //Time between Sensor Readings in milliseconds
  unsigned long previousMillisVoltage = 0;
  unsigned long currentMillisVoltage;


  //Serial Communication
  
  #define startMarker 0x78   //Marks Beggging of Datastream   ASCII for x
  #define endMarker 0x71    //Marks End of Datastream   ASCII for q
  #define maxMessage 256   //Number of Bytes that can be transmitted in one message

  bool inProgress = false;
  bool allReceived = false;
  bool DataSent = false;
  
  byte bytesRecvd = 0;
  byte dataRecvCount = 0; 

  char tempBufferIn[maxMessage];
  char tempBufferOut[maxMessage];
  
  int nb = 0;

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
  float IndicatorSize = 0.20;   //Size of each LED-Strip Segment

  //Define Reverse Light
  int endspace = 0;   //Distance from reverslights to end of strip
  int revlightsize = NUM_LEDS_BACK * IndicatorSize;    //Size of Reverslight on each side

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

  int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
  int IndicatorOnTime = 250;
  int IndicatorOffTime = 500; //Time between eacht indicator animation

  //Delay Variables

  int StartupAnimTime = 25; //Time between each LED activatiung at startup
  int StartupFadeTime = 1000; //Time for Fade at the end of startup
  


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
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

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

  //Voltage Sensors

  pinMode(VoltagePin[0], INPUT);
  pinMode(VoltagePin[1], INPUT);
  pinMode(VoltagePin[2], INPUT);
  pinMode(VoltagePin[3], INPUT);
  pinMode(VoltagePin[4], INPUT);

  for(int i = 0; i < 5; i++) {
    InputVoltage[i] = MaxVoltage / ((R1[i] + R2[i])/R2[i]);     //Calculate Voltage which is applied to GPIO at MaxVoltage    
    if(InputVoltage[i] > LogicLevel) {   //If the Voltage applied to the GPIOS can be too high, set Error HIGH
      Error[7+i] = 1;
    }
    else {
      Error[7+i] = 0;
    }
  }

  for(;;){
    Task1loop();   
  } 
}

void Task1loop() {
  
  CheckError();
  ReadTemp();
  ReadSonar();
  ReadIMU();
  Fan_Control();
  VoltageSensor();
  ConvertVarToString();
  getSerialData(1);
  ConvertStringtoVar();

  delay(1);
}

void ReadTemp() {

  currentMillisTemp = millis();
  if(currentMillisTemp - previousMillisTemp >= ReadIntervallTemp) {
    previousMillisTemp = currentMillisTemp;
  
    Error[6] = 0;
    
    for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them
      SaveTemp(SensorPins[i], i);
      //Serial.print("Temp");
      //Serial.print(i + 1);
      //Serial.print(": ");
      //Serial.print(temp[i][TempUnit]);
      //Serial.print(" C Error: ");
      //Serial.println(Error[i]);
  
      if(Error[i] == 1) {
        Error[6] = 1;              
      }
      
    }
  
    if(Error[6] == 1) {
      digitalWrite(ErrorLedPin, HIGH);
    }
    else {
      digitalWrite(ErrorLedPin, LOW);
    }
    
    //Serial.println(" ");
    
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
      //Serial.print(SonarValues[i]); Serial.print("   ");
      
    }
    //Serial.println();
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
    
    //Serial.print("AcX = "); Serial.print(AcX);
    //Serial.print(" | AcY = "); Serial.print(AcY);
    //Serial.print(" | AcZ = "); Serial.print(AcZ);
    //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    //Serial.print(" | GyX = "); Serial.print(GyX);
    //Serial.print(" | GyY = "); Serial.print(GyY);
    //Serial.print(" | GyZ = "); Serial.println(GyZ);
    //Serial.println("  ");
    
  }    
  
}

void Fan_Control() {

  for(int i = 0;i < 4;i++) {

    DutyCycle[i] = map(FanSpeed[i], 0, 100, 0, pow(2, Resolution[i]));

    ledcWrite(PWMChannel[i], DutyCycle[i]);

  }

}

void VoltageSensor() {
  
  currentMillisVoltage = millis();
  if(currentMillisVoltage - previousMillisVoltage >= ReadIntervallVoltage) {
    previousMillisVoltage = currentMillisVoltage;

    for(int i = 0;i < NOSVoltage;i++) {                         
      float value = analogRead(VoltagePin[i]);                              //Reads all the AnalogPins Values
      Voltage[i] = value * (LogicLevel/pow(2, ADCRes)) * ((R1[i] + R2[i])/R2[i]);    //Calculates the voltages from the sensorpins values
    }
    
  /*for(int i = 0;i < NOSVoltage;i++) {   //Prints all Voltages to Serial monitor
      Serial.print(Voltage[0]);
      Serial.print(" ");
    }
  */}
}

void ConvertVarToString() {   //Converts Data from Variables to a String to be sent

    //int NOSVoltage = 5;     //DEBUG ONLY
    //float Voltage[5] = {12.1, 56.4, 89.2, 23.5, 74.2};    //DEBUG ONBLY
  
  char BufferSonar[128];
  char BufferIMU[64];
  char StringIMUTemp[7];
  char StringVoltage[5][5];
  char BufferVoltage[32];
  char StringTemp[6][6];
  char BufferTemp[32];
  
  
  for(int i = 0; i < NOSVoltage; i++) {           //Converts Voltage Floats to String
    dtostrf(Voltage[i], 3, 1, StringVoltage[i]);
  }
  
  for(int i = 0; i < NrOfSensors; i++) {           //Converts Temp Floats to String
    dtostrf(temp[i][TempUnit], 3, 1, StringTemp[i]);
  }
  
  float floatTemp = Tmp/340.00+36.53;
  
  dtostrf(floatTemp, 3, 1, StringIMUTemp);
  
  sprintf(BufferSonar, "x%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,", SonarValues[0], SonarValues[1], SonarValues[2], SonarValues[3], SonarValues[4], SonarValues[5], SonarValues[6], SonarValues[7], SonarValues[8], SonarValues[9], SonarValues[10], SonarValues[11], SonarValues[12], SonarValues[13], SonarValues[14], SonarValues[15]);    //convert all Variables into substrings
  sprintf(BufferIMU, "%i,%i,%i,%s,%i,%i,%i,", AcX, AcY, AcZ, StringIMUTemp, GyX, GyY, GyZ);
  sprintf(BufferVoltage, "%s,%s,%s,%s,%s,", StringVoltage[0], StringVoltage[1], StringVoltage[2], StringVoltage[3], StringVoltage[4]);
  sprintf(BufferTemp, "%s,%s,%s,%s,%s,%s,%huq", StringTemp[0], StringTemp[1], StringTemp[2], StringTemp[3], StringTemp[4], StringTemp[5], IsError);
  sprintf(tempBufferOut, "%s%s%s%s", BufferSonar, BufferIMU, BufferVoltage, BufferTemp);   //Combine all substrings into one
  
  //Serial.println(tempBufferOut);
}

void ConvertStringtoVar() {   //converts incoming Serial Data String to Variables
  
  //unsigned int TempUnitBuffer;
  unsigned int IsErrorPandaBuffer;

  sscanf(tempBufferIn, "x%u,%x,%x,%u,%u,%u,%u,%u,%uq", &Effekt, &EffektColor1, &EffektColor2, &FanSpeed[0], &FanSpeed[1], &FanSpeed[2], &FanSpeed[3], &TempUnit, &IsErrorPandaBuffer);
  
  //if(TempUnitBuffer > 1) {
  //  TempUnit = TempUnitBuffer;
  //}
  
  if(IsErrorPandaBuffer > 1) {
    IsErrorPanda = IsErrorPandaBuffer;
  }
  
  //Serial.print(Effekt); Serial.print(" "); Serial.print(EffektColor1, 16); Serial.print(" "); Serial.print(EffektColor2, 16); Serial.print(" "); Serial.print(FanSpeed[0]); Serial.print(" "); Serial.print(FanSpeed[1]); Serial.print(" "); Serial.print(FanSpeed[2]); Serial.print(" ");    //For Debug Only
  //Serial.print(FanSpeed[3]); Serial.print(" "); Serial.print(TempUnit); Serial.print(" "); Serial.println(IsErrorPanda);
  
}

void CheckError() {   //Checks if there has been an Error

  IsError = false;  

  for (int i = 0; i < ErrorCnt; i++) {
    
    if (Error[i]) {  
      
      IsError = true;  
       
    }
    
  }

}

void getSerialData(bool printout) { 	  //If printout = 1 sends back data, 0 doesnt send back data
  
  while (Serial.available() > 0) {    //Checks for Serial Data
    
    byte x = Serial.read();   //Reads Serial Data
    
    if (x == startMarker && inProgress == false) {    //IF the first Byte = startMarker, start to recieve Data
       
      bytesRecvd = 0; 
      inProgress = true;
      
    }
    
    if (inProgress ) {
       
      if (bytesRecvd<maxMessage) {
        
        tempBufferIn[bytesRecvd++] = x;
        
      }
      
      if (x == endMarker) { 
        
        inProgress = false;
        allReceived = true;
        nb = bytesRecvd;
        
        if(printout == 1) {
          Serial.print(tempBufferOut);    //Send Data Back to LattePanda
        }
      }
      
    }
    
  }
  
}

////////// C O R E 2 //////////

void Task2setup( void * pvParameters ){    //Task2 Core 1
  //Serial.print("Task2 running on core ");
  //Serial.println(xPortGetCoreID());

  delay(1000);

  FastLED.addLeds<WS2812B, LED_PIN_BACK, GRB>(ledsback, NUM_LEDS_BACK);  //Initlialize Back LEDStrip
  FastLED.addLeds<WS2812B, LED_PIN_FRONT, GRB>(ledsfront, NUM_LEDS_FRONT);  //Initialize Front LEDStrip  
    
  pinMode(LED_PIN_BACK, OUTPUT);              //Declare Pins Input/Output
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

  startup();    //Run Startup Animation

  for(;;){
    Task2loop();
  }
}

void Task2loop() {  

  //ReadButtons();
  //Effect();
  
  delay(1);
}

void startup() {

  int y = NUM_LEDS_BACK / 2;
  int z= 0;
  
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

  FadeToColor(brakecol, idlecolback, StartupFadeTime, 0, NUM_LEDS_BACK); 
  
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
      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = false; 
      break;

    case 2:   //Left indicator

      BrakeLight();
      Indicator(1);
      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = false;
      ReverseLightState = false; 
      break;

    case 3:   //Brakelight

      BrakeLight();   
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false;    
      break;

    case 4:   //Reverse Light

      BrakeLight();
      ReverseLight();
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;
    
    case 5:   //Hazard Light

      Indicator(2);   
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = false; 
      HazardState = true;             
      break;

    case 6:  //Right Indicator + Brake Light    

      BrakeLight();
      Indicator(0);
      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false; 
      break;

    case 7:  //Left Indicator + Brake Light

      BrakeLight();
      Indicator(1);
      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = true;
      ReverseLightState = false; 
      break;      

    case 8:  //Hazard Light + Brake Light

      BrakeLight();
      Indicator(2);
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false; 
      HazardState = true;
      break;

    case 9:   //Reverse Light + Brake Light
    
      BrakeLight();
      ReverseLight();
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;
    
    case 10:  //Right Indicator + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(0);
      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;    

    case 11:  //Left Indicator + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(1);
      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;

    case 12:  //Hazard Lights + Reverse Light

      BrakeLight();
      Indicator(2);  
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true;   
      HazardState = true;  
      break;

    case 13:  //Right Indicator + Brake Light + Reverse Light
      
      BrakeLight();
      ReverseLight();
      Indicator(0);
      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;

    case 14:  //Left Indicator + Brake Light + Reverse Light

      BrakeLight();
      ReverseLight();
      Indicator(1);
      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;

    case 15:  //Hazard Lights + Brake Light + Reverse Light
    
      BrakeLight();
      Indicator(2);
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true;
      HazardState = true; 
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
  //IndicatorRightState = digitalRead(ButtonPins[0]);
  //IndicatorLeftState = digitalRead(ButtonPins[1]);
  //BrakeLightState = digitalRead(ButtonPins[2]);
  //ReverseLightState = digitalRead(ButtonPins[3]);
  //HazardState = digitalRead(ButtonPins[4]);

   //Turns on effects based on Effect Variable, Brake Light is also called when not in use to turn it off
  switch(Effekt) {
    case 0:   //Normal Light

      break;

    case 1:   //Right indicator

      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = false; 
      break;

    case 2:   //Left indicator

      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = false;
      ReverseLightState = false; 
      break;

    case 3:   //Brakelight

      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false;    
      break;

    case 4:   //Reverse Light

      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;
    
    case 5:   //Hazard Light

      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = false; 
      HazardState = true;             
      break;

    case 6:  //Right Indicator + Brake Light    

      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false; 
      break;

    case 7:  //Left Indicator + Brake Light

      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = true;
      ReverseLightState = false; 
      break;      

    case 8:  //Hazard Light + Brake Light


      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = false; 
      HazardState = true;
      break;

    case 9:   //Reverse Light + Brake Light
    

      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;
    
    case 10:  //Right Indicator + Reverse Light


      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;    

    case 11:  //Left Indicator + Reverse Light

      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = false;
      ReverseLightState = true; 
      break;

    case 12:  //Hazard Lights + Reverse Light
 
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = false;
      ReverseLightState = true;   
      HazardState = true;  
      break;

    case 13:  //Right Indicator + Brake Light + Reverse Light

      IndicatorRightState = true;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;

    case 14:  //Left Indicator + Brake Light + Reverse Light

      IndicatorRightState = false;
      IndicatorLeftState = true;
      BrakeLightState = true;
      ReverseLightState = true; 
      break;

    case 15:  //Hazard Lights + Brake Light + Reverse Light
    
      IndicatorRightState = false;
      IndicatorLeftState = false;
      BrakeLightState = true;
      ReverseLightState = true;
      HazardState = true; 
      break;     
       
  }
  
}


void idle() {

  for(int i = 0;i <= NUM_LEDS_BACK;i++) {    //State of LEDs when nothing is happening
    ledsback[i] = idlecolback;
    SwitchFrontLedColor(i,idlecolfront,2);
  }
   
   FastLED.show();
   
}

void Indicator(int dir) {    //dir = 1 for left, dir = 0 for right, dir = 2 for both

  

  if(dir == 1) {    //Left Indicator
  
    int z = 0;
    
    for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0;i--) {   //Animation 
      ledsback[i] = indicatorcol; 
      SwitchFrontLedColor(z,indicatorcol,1);
      FastLED.show();   
      z++;
      //delay(IndicatorAnimTime);
      mydelay(IndicatorAnimTime);             
    }

    //delay(IndicatorOnTime);
    mydelay(IndicatorOnTime);
    
    z = 0;
    
    for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0;i--) {
      ledsback[i] = idlecolback;
      SwitchFrontLedColor(z,idlecolfront,1);
      z++;               
    }
    FastLED.show();
    //delay(IndicatorOffTime);
    mydelay(IndicatorOffTime);   

  }

  if(dir == 0) {    //Right Indicator 
  
    int z = 0;
  
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {    //Animation
      ledsback[i] = indicatorcol;
      SwitchFrontLedColor(z,indicatorcol,0);
      FastLED.show();
      z++;
      //delay(IndicatorAnimTime);
      mydelay(IndicatorAnimTime);                
       
    }    

    z = 0;

    //delay(IndicatorOnTime);
    mydelay(IndicatorOnTime);
        
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {
      ledsback[i] = idlecolback;
      SwitchFrontLedColor(z,idlecolfront,0);
      z++;
    }
    
    FastLED.show();
    
    //delay(IndicatorOffTime);
    mydelay(IndicatorOffTime);

  }

  if(dir == 2) {    //Hazard Lights

    int y = NUM_LEDS_BACK * IndicatorSize;   //Variable for Right Indicator
    int z= 0;
    
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {  //i used for left, y used for right side
      ledsback[i] = indicatorcol;
      ledsback[y] = indicatorcol;
      SwitchFrontLedColor(z,indicatorcol,3);
      FastLED.show();
      y--;
      z++;
      //delay(IndicatorAnimTime);
      mydelay(IndicatorAnimTime);
      
    }    

    //delay(IndicatorOnTime);
    mydelay(IndicatorOnTime);  
        
    y = NUM_LEDS_BACK * IndicatorSize;     //Reset y
    z = 0;
    
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {    //Turn everything Black
      ledsback[i] = idlecolback;
      ledsback[y] = idlecolback;
      SwitchFrontLedColor(z,idlecolfront,3);
      y--;
      z++;
    }
    
    FastLED.show();
    //delay(IndicatorOffTime);
    mydelay(IndicatorOffTime);

  } 

     
}

void BrakeLight() {


  //getSerialData(0);
  //ConvertStringtoVar();  
  UpdateButtonState();

  ReverseLight();

  //If BrakeLight isnt supposed to be turned on, turn it off
  if(BrakeLightState == LOW) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = idlecolback;              
    }      
  }
  
  else {

    //Lights up Center of Strip
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = brakecol;
    }
  }  
  
  //Checks if Left Indicator is running, if not lights up left side of Strip
  if(IndicatorLeftState == LOW && ReverseLightState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
    for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = brakecol;
    }
  } 

  else if(IndicatorLeftState == HIGH || HazardState == HIGH || ReverseLightState == HIGH) {
    
  }

  else {
    for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = idlecolback;
    }    
  }

  //Checks if Right Indicator is running, if not lights up right side of the Strip
  if(IndicatorRightState == LOW && ReverseLightState == LOW && HazardState == LOW && BrakeLightState == HIGH) {
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
      ledsback[i] = brakecol;
    }
  }   

  else if(IndicatorRightState == HIGH || HazardState == HIGH || ReverseLightState == HIGH) {
    
  }
  
  else {
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
      ledsback[i] = idlecolback;
    }    
  }

  FastLED.show();
}

void ReverseLight() {  

  if(IndicatorRightState == LOW && HazardState == LOW && ReverseLightState == HIGH) {  
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <=NUM_LEDS_BACK;i++) {
      ledsback[i] = reversecol;
    }
  }

  if(IndicatorLeftState == LOW && HazardState == LOW && ReverseLightState == HIGH) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0 ;i--) {
      ledsback[i] = reversecol;
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

void FadeToColor(unsigned long Color1, unsigned long Color2, unsigned int timeinms, int ledbegin, int ledend) {     //This function fades all the LEDs from Color1 to Color 2 in given time "timeinms" Color1 and Color2 should be HEX Values format: "0x000000"

  //Convert Color1 froim HEX to RGB this is done by splitting the HEX Value in 3 Parts and converting it do decimal
  int r1 = Color1 >> 16;

  int g1 = (Color1 & 0x00ff00) >> 8;

  int b1 = (Color1 & 0x0000ff);

  //Convert Color2 from HEX to RGB
  int r2 = Color2 >> 16;

  int g2 = (Color2 & 0x00ff00) >> 8;

  int b2 = (Color2 & 0x0000ff);
  
  //Calsculate Differences between Color1 and Color2 RGB Values
  
  unsigned int d1 = abs(r1 -r2);
  
  unsigned int d2 = abs(g1 - g2);
  
  unsigned int d3 = abs(b1 -b2);
 
  //Determine how many steps it takes to Fade from Color1 to Color2, the biggest differnce of the RBB Values is taken
  int steps = 0;
  
  if(d1 >= d2 || d1 >= d3) {
    steps = d1;
  }
  
  else if(d2 >= d1 || d2 >= d3) {
    steps = d2;
  }
  
  else if(d3 >= d1 || d3 >= d2) {
    steps = d3;
  }
  
  //For Loop to fade the colors
  if(steps != 0) {
    for(int i = 0; i <= steps; i++) {    
    
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
      
      delay(timeinms / steps);
      
    }
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
  
  if(dir == 3) {  //Both sides will be effected expect the bottom row of leds
    
    ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    
  }
  
}

