#include <Arduino.h>
#include "LattePandacomms.h"

#define _maxMessage 512

char BufferIn[_maxMessage] = "x3,10,1";    //Buffer for incoming messages
char BufferOut[_maxMessage];   //Buffer for outgoing messages

LattePandacomms::LattePandacomms() {
  
  
  
}

bool LattePandacomms::refresh(char startMarker, char endMarker) {
  
  _inProgress = false;
  
  while (Serial.available() > 0) {    //Checks for Serial Data
    
    byte x = Serial.read();   //Reads Serial Data
    
    if (x == startMarker && _inProgress == false) {    //IF the first Byte = startMarker, start to recieve Data
       
      _bytesRecvd = 0;      //Zero bytes have been recieved so far
      _inProgress = true;   //New Message has started
      
      for(int i = 0; i < sizeof(BufferIn); i++) {   //Clear BufferIn Array to make space for new message
        BufferIn[i] = '\0';
      }
      
    }
    
    if (_inProgress ) {   //Has the message started?
          
      if (_bytesRecvd<_maxMessage) {    //Does the current length exceed the max Message limit
        
        BufferIn[_bytesRecvd++] = x;
        
      }
      
      if (x == endMarker) { 
        
        _inProgress = false;
        //allReceived = true;
        _nb = _bytesRecvd;
        
      }
      
    }
    
  }
  
  //Serial.println(BufferIn);   //DEBUG
  return 0;
  
}

bool LattePandacomms::decode() {
  
  int Type;
  int valCount;
  int Access;
  
  sscanf(BufferIn, "x%u,%u,%u", &Type, &valCount, &Access);

  Serial.println(Type);
  Serial.println(valCount);
  Serial.println(Access);
  
  switch(Access) {
    
    case 0:   //Read only || Write to ESP
    
      if(_ExpectedAccess[Type] == Access) {   //Check if Access corresponds to Type of message
        
      }
      
      else {    //If not return Error
        _commsError = 1;
      }
      
    break;
    
    case 1:   //Write only || Send to LattePanda
    
      if(_ExpectedAccess[Type] == Access) {   //Check if Access corresponds to Type of message
        
      }
      
      else {    //If not return Error
        _commsError = 1;
      }
    
    break;
    
    case 2:   //Read & Write || Write to ESP and send to LattePanda
    
      if(_ExpectedAccess[Type] == Access) {   //Check if Access corresponds to Type of message
        
      }
      
      else {    //If not return Error
        _commsError = 1;
      }
    
    break;
    
  }
  
  return 0;
  
}

bool LattePandacomms::send() {
  
  
  
  return 0;
  
}