//Reciever

HardwareSerial Reciever(1);

void setup() {
  
  Reciever.begin(115200, SERIAL_8N1, 25, 26);
  Serial.begin(115200);
}

void loop() {
  
  while(Reciever.available()){
    int RxdFloat = Reciever.read();
    Serial.println(RxdFloat);
  }

}
