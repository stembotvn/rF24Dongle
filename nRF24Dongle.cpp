#include "nRF24Dongle.h"

void nRFDongle::init(){
Serial.begin(115200);
//SPI.begin();
radio.begin();
network.begin(108,MasterNode);// Channel Frequency = 108, Node add = 0 (master node)
}
/////////////////////////
void nRFDongle::set_address(uint16_t nodeAddr){
	_slaveNode = nodeAddr; 
}
//////////////////////////
void nRFDongle::readSerial(){
isAvailable = false; 
if (Serial.available() > 0){
  isAvailable = true; 
  serialReadbyte = Serial.read();
  }
if (isAvailable) {
   unsigned char c = serialRead&0xff;
    if(c==0x55&&isStart==false){
     if(prevc==0xff){
      index=1;
      isStart = true;
      //buffer[index]=c;
    }
    }else{
      prevc = c;  
      if(isStart){
        if(index==2){
         dataLen = c; 
         payloadLen = dataLen; 
        }else if(index>2){
          dataLen--;
        }
      }
    } 
     buffer[index]=c;
     index++;
     if(index>51){
      index=0; 
      isStart=false;
     }
     if(isStart&&dataLen==0&&index>3){ 
        isStart = false;
         State = SERIAL_PARSING; // Serial Data available now, State change to parsing 
        index=0;
     }
  }
}/////
void nRFDongle::callOK(){   //sending 0xff 0x55 /r /n
 Serial.write(0xff);
 Serial.write(0x55);
 Serial.println();
}
///////
void nRFDongle::parsingSerial(){
int action = buffer[4];
  int action_type = buffer[5];
  switch(action){
    case  GET: State = RF_WRITE; 
    break;
     case RUN:{
       if (action_type == SET_ADDRESS) { // setting target address for Robot; 
         _slaveNode = buffer[6];
         callOK();
         State = SERIAL_CHECK; 
         }
        else State = RF_WRITE;  

     }
      break;
      case RESET:{
        //reset
         callOK();
        State = SERIAL_CHECK; 
      }
     break;
     case START:{
        //start
        callOK();
        State = SERIAL_CHECK; 

      }
     break;
  }
}

///////////////////////////////////////////////////////////////
void nRFDongle::writeRF(){
RF24NetworkHeader header(_slaveNode,'T');
bool OK = network.write(header,buffer,payloadLen+2);
if (OK) State = RF_READ;  //if onnect and send successfully 
else {
   callOK();    
   State = SERIAL_CHECK; 
 }
}
///////////////////////////////////////////////////////////
void nRFDongle::readRF(){
network.update(); 
RFread_size = 0;
while ( network.available() )  {
  RF24NetworkHeader header;
  network.peek(header);
  if(header.type == 'T'){
    RFread_size = network.read(header,RFbuf,MAX_READ_SIZE);
    }
  if (RFread_size > 1) State = SERIAL_SEND;
  else {
   callOK();    
   State = SERIAL_CHECK; 
  } 
 }
}
////////
void nRFDongle::sendSerial(){
for (int i = 0;i<RFread_size;i++) {
  Serial.write(RFbuf[i]);
  }
State = SERIAL_CHECK;  
}

////
void nRFDongle::run(){
switch (State) {

  case SERIAL_CHECK :
  readSerial(); 
  break; 

  case SERIAL_PARSING :
  parsingSerial();
  
  break;
  
  case RF_WRITE :
  writeRF();
  break;
 
  case RF_READ :
  readRF();
  break; 

  } 	
}