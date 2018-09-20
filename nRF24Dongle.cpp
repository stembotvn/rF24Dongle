#include "nRFDongle.h"

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
        State = RFwrite; // Serial Data available now, State change to RF write in next loop 
        index=0;
     }
  }
}
///
nRFDongle::parsing_Serial(){
  
}
/*
ff 55 len idx action device port  slot  data a
0  1  2   3   4      5      6     7     8
*/
//////////////////////////
void nRFDongle::writeRF(){
RF24NetworkHeader header(_slaveNode,'T');
bool OK = network.write(header,buffer,payloadLen+2));
if (OK) State = RFread; 
else State = Ending; 
}
//////////////////////////
void nRFDongle::readRF(){
network.update(); 
while ( network.available() )  {
  RF24NetworkHeader header;
  network.peek(header);
  if(header.type == 'A'){
    network.read(header,&time,sizeof(time));
    Serial.print("Got time: ");
    Serial.println(time);
  }
}
}
////////
void nRFDongle::sendSerial(){

}
////////
void nRFDongle::endProcess(){

}
////
void nRFDongle::run(){
switch (State) {
  case SerialCheck :
  readSerial(); 
  break; 
  case RFwrite :
  writeRF();
  break;
  case networkUpdate :

  break; 
  case RFread :

  break; 
  case RFreadParsing :

  break;
  case SerialSend :

  break; 
  case Ending :
  endProcess(); 
  break; 

} 	
}