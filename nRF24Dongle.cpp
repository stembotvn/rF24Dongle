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

}
//////////////////////////
void nRFDongle::writeRF(){

}
//////////////////////////
void nRFDongle::readRF(){

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