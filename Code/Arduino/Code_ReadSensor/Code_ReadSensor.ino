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

const int ErrorLedPin = 9; //Pin for Error LED

const unsigned int ReadIntervallTemp = 200; //Time between Sensor Readings in milliseconds
unsigned long previousMillisTemp = 0;
unsigned long currentMillisTemp;

float temp[NrOfSensors + 1][3];

bool Error[NrOfSensors];
bool IsError = 0;

void setup(){

  Serial.begin(115200);
  myOcto.begin(ACTIVE_SONARS);   // initialize bus, pins etc

}

void loop(){

  SendTemp();
  ReadSonar();
  
}

void ReadTemp(int Pin, int Pos) {   //Pin of Sensor and Position of Value in temp Array
  
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

void SendTemp() {

  currentMillisTemp = millis();
  if(currentMillisTemp - previousMillisTemp >= ReadIntervallTemp) {
    previousMillisTemp = currentMillisTemp;
  
    IsError = 0;
    
    for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them
      ReadTemp(SensorPins[i], i);
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
