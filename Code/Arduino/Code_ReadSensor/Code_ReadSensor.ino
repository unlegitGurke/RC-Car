//Create TaskHandles

TaskHandle_t Task1;
TaskHandle_t Task2;

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

void Task1setup( void * pvParameters ){    //Task1 Core 0
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  myOcto.begin(ACTIVE_SONARS);   // initialize bus, pins etc

  pinMode(ErrorLedPin, OUTPUT);

  for(;;){
    Task1loop();   
  } 
}

void Task1loop() {
  
  ReadTemp();
  ReadSonar();
  ReadIMU();

  delay(1);
}

void Task2setup( void * pvParameters ){    //Task2 Core 1
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    Task2loop();
  }
}

void Task2loop() {
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
      Serial.print("°C Error: ");
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
