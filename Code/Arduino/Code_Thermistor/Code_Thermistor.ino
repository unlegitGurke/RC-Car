const int NrOfSensors = 6;    //Define Number of Sensors connected to the Analog Pins of the Arduino.

const int TempUnit = 2;   //Choose which int which Unit the Temperature will be printed. 1 for Kelvin, 2 for Celcius, 3 for Fahrenheit.

float temp[NrOfSensors + 1][3];

void setup(){

  Serial.begin(9600);

}

void loop(){

  for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them.
    ReadTemp(i);
    Serial.print("Temp");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(temp[i][TempUnit]);
  }
  
  Serial.println("");
  delay(500);
  
}

void ReadTemp(int Pin) {
  
  int tempReading = analogRead(Pin);
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit

  temp[Pin][1] = tempK;
  temp[Pin][2] = tempC;
  temp[Pin][3] = tempF;
  
}
