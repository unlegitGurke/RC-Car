//Reciever

HardwareSerial Debug(2);

#define startMarker 0x78   //Marks Beggging of Datastream   ASCII for x
#define endMarker 0x71    //Marks End of Datastream   ASCII for q
#define maxMessage 256   //Number of Bytes that can be transmitted in one message

bool inProgress = false;
bool allReceived = false;
bool DataSent = false;

bool ButtonState = false;
bool lastButtonState = false;

byte bytesRecvd = 0;
byte dataRecvCount = 0; 

byte tempBuffer[maxMessage];

int nb = 0;   //Number of Bytes currently saved into tempBuffer

void setup() {
  
  Debug.begin(115200, SERIAL_8N1, 17, 16);
  Serial.begin(115200);
  pinMode(18, INPUT);
  
}

void loop() {
  
  ButtonState = digitalRead(18);
  
  if(ButtonState == HIGH && lastButtonState == LOW && DataSent == false) {
    
    Serial.print("x0,FFFFFF,FFFFFF,25,50,75,100,2,0q"); //First packet to be sent
    DataSent = true;
    
  }
  
  lastButtonState = ButtonState;
    
  getSerialData();
    
  if(allReceived == true) {
    
    Debug.write(tempBuffer, nb);
    allReceived = false;
    
  }
    
}

void getSerialData() {
  
  while (Serial.available() > 0) {    //Checks for Serial Data
    
    byte x = Serial.read();   //Reads Serial Data
    
    if (x == startMarker && inProgress == false) {    //IF the first Byte = startMarker, start to recieve Data
       
      bytesRecvd = 0; 
      inProgress = true;
      
    }
    
    if (inProgress ) {
       
      if (bytesRecvd<maxMessage) {
        
        tempBuffer[bytesRecvd++] = x;
        
      }
      
      if (x == endMarker) { 
        
        inProgress = false;
        allReceived = true;
        nb = bytesRecvd;
        
      }
      
    }
    
  }
  
}
