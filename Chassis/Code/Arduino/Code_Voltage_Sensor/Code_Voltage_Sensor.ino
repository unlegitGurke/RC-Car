const float R1 = 64900;   //Voltage Divider Resistor Values
const float R2 = 4300;

const int NOSVoltage = 5;   //Number of Sensors plugged in
const int VoltagePin[] = {A0, A1, A2, A3, A4};  //Sensor Pins
const int ADCRes = 10;    //Resolution of the Analog Digital Converters
const float LogicLevel = 5;   //Logic Level of the microcontroller
const float MaxVoltage = 80;  //Max Voltage that will be measured

float Voltage[NOSVoltage];  //Voltage Data will be saved in here
bool Error = 0;             //If this bool is HIGH, MAXVoltage is too high for chosen resistors, Microcontroller may be harmed
float InputVoltage = 0;

void setup() {
  
  Serial.begin(9600);
  pinMode(VoltagePin[0], INPUT);
  pinMode(VoltagePin[1], INPUT);
  pinMode(VoltagePin[2], INPUT);
  pinMode(VoltagePin[3], INPUT);
  pinMode(VoltagePin[4], INPUT);
  
  InputVoltage = MaxVoltage / ((R1 + R2)/R2);     //Calculate Voltage which is applied to GPIO at MaxVoltage
  
  if(InputVoltage > LogicLevel) {   //If the Voltage applied to the GPIOS can be too high, set Error HIGH
    Error = 1;
  }
  else {
    Error = 0;
  }
  
}

void loop() {
  
  for(int i = 0;i < NOSVoltage;i++) {                         
    float value = analogRead(VoltagePin[i]);                              //Reads all the AnalogPins Values
    Voltage[i] = value * (LogicLevel/pow(2, ADCRes)) * ((R1 + R2)/R2);    //Calculates the voltages from the sensorpins values
  }
  
  for(int i = 0;i < NOSVoltage;i++) {   //Prints all Voltages to Serial monitor
    Serial.print(Voltage[0]);
    Serial.print(" ");
  }
  
  Serial.println(Error);
  
}