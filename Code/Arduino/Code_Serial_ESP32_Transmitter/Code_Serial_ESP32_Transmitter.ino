//Transmitter

HardwareSerial Transmitter(1);

#define Poti 32

void setup() {

  pinMode(Poti ,INPUT);
  Transmitter.begin(115200, SERIAL_8N1, 25, 26);
  Serial.begin(115200);

}

void loop() {

  while(Transmitter.available()){
    int PotiPoint = analogRead(Poti);
    //PotiPoint = map(PotiPoint, 0, 4095, 0, 255);
    Transmitter.write(PotiPoint);
    Serial.println(PotiPoint); 
    delay(1);
  }

}
