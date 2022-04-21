const float temprangemin = 218.15;    //Temperature Range of thermistor in Kelvin
const float temprangemax = 398.15;

const int NrOfSensors = 6;    //Define Number of Sensors connected to the Analog Pins of the Arduino

const int TempUnit = 2;   //Choose which Unit the Temperature will be printed in, 1 for Kelvin, 2 for Celcius, 3 for Fahrenheit

const int ErrorLedPin = 13;

float temp[NrOfSensors + 1][3];

bool Error[NrOfSensors];
bool IsError = 0;

void setup(){

  Serial.begin(9600);

}

void loop(){

  IsError = 0;
  
  for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them
    ReadTemp(i);
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
  delay(500);
  
}

void ReadTemp(int Pin) {
  
  int tempReading = analogRead(Pin);
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit

  if(tempK > temprangemin && tempK < temprangemax) {
    temp[Pin][1] = tempK;
    temp[Pin][2] = tempC;
    temp[Pin][3] = tempF;    
    Error[Pin] = 0;
  }
  else {
    temp[Pin][1] = 0;
    temp[Pin][2] = 0;
    temp[Pin][3] = 0;    
    Error[Pin] = 1;
  }
  
}
