//Transmitter

HardwareSerial Debug(2);

#define startMarker 0x78   //Marks Beggging of Datastream,   ASCII for x
#define endMarker 0x71    //Marks End of Datastream,   ASCII for q
#define maxMessage 256   //Number of Bytes that can be transmitted in one message

bool newData = false;
bool inProgress = false;
bool startFound = false;
bool allReceived = false;

byte bytesRecvd = 0;
byte dataRecvCount = 0; 

char dataRecvd[maxMessage]; 
char tempBuffer[maxMessage];

int nb = 0;   //Number of Bytes currently saved into tempBuffer

void setup() {

  Debug.begin(115200, SERIAL_8N1, 17, 16);
  Serial.begin(115200);

}

void loop() {
  
  getSerialData();
  
}

void getSerialData() {
  
  while (Serial.available() > 0) {    //Checks for Serial Data
    
    byte x = Serial.read();   //Reads Serial Data
    
    if (x == startMarker && inProgress == false) {    //IF the first Byte = startMarker, start to recieve Data
       
      bytesRecvd = 0; 
      inProgress = true;
      Debug.print("x");
      
    }
    
    if (inProgress ) {
       
      if (bytesRecvd<maxMessage) {
        
        tempBuffer[bytesRecvd++] = x;
        
      }
      
      if (x == endMarker) { 
        
        inProgress = false;
        allReceived = true;
        nb = bytesRecvd;
        Debug.print("q");
        Serial.print("x001,002,003,004,005,006,007,008,009,010,011,012,013,014,015,016,0001,0002,0003,0004,0005,0006,0007,001,002,003,004,005,0001,0002,0003,0004,0005,0006q"); //Packet which  needs to be sent back
        Debug.print("Sent");
        
      }
      
    }
    
  }
  
}
