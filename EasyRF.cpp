#include "EasyRF.h"
//#include "RF24.h"

/*
EasyRF::EasyRF(int CE_PIN,int CSN_PIN) {
_CE = CE_PIN;
_CSN = CSN_PIN; 
}*/
EasyRF::EasyRF(RF24& _radio) : radio(_radio) {

}
////
bool EasyRF::init(uint16_t myaddress){
    bool OK = false;
my_node = myaddress; 
#ifdef DEBUG
Serial.begin(115200);
Serial.println("NRF init...");
#endif
OK=radio.begin();

if (radio.isChipConnected())
    {
isRF24Connected=true;     
radio.setChannel(myChannel); 
radio.setPALevel(rfPower);
if (dynPayload_en){ 
radio.enableDynamicPayloads();
}
radio.setDataRate(rfSpeed);
radio.setRetries(retryDelay,retryTimes);
radio.setCRCLength( crcLen );
radio.openReadingPipe(1,convert_address(my_node));
//radio.openReadingPipe(multiCast_channel,convert_address(multiCast_node));
//radio.setAutoAck(multiCast_channel,false);	
radio.startListening();
#ifdef DEBUG
Serial.println("NRF READY");
//radio.printDetails();                   // Dump the configuration of the rf unit for debugging
#endif 
    }
else
     {
    #ifdef DEBUG 
Serial.println("Could not find NRF24L01. CHECK NRF Module connection");
    #endif
    isRF24Connected = false;
    }
    delay(1000);
    return OK;
}
/////
void EasyRF::RFpowerDown(){
    if (radio.isChipConnected()) radio.powerDown();
}
void EasyRF::RFpowerUp(){
if (radio.isChipConnected())  radio.powerUp();
  
}
/////
void EasyRF::Multicast_readingStart(){
    radio.openReadingPipe(multiCast_channel,convert_address(multiCast_node));
    radio.startListening();
    radio.setAutoAck(multiCast_channel,false);	

}

///
void EasyRF::SetAddress(uint16_t myaddress){
    my_node = myaddress; 
    radio.stopListening();
    #ifdef DEBUG
    uint64_t ad = convert_address(myaddress);
    Serial.print("Convert Address from 16bit: ");
    Serial.print(myaddress,HEX);
    Serial.print("  to 40bit: ");
    //Serial.println(ad);
    #endif 
    //address = convert_address(my_node);
    radio.openReadingPipe(1,convert_address(my_node));
    radio.startListening();
}

////
void EasyRF::SetMultiCastAddress(uint16_t addr,uint8_t ch){
multiCast_node = addr;
multiCast_channel = ch;
radio.stopListening();
radio.openReadingPipe(multiCast_channel,convert_address(multiCast_node));
radio.startListening();
}
////

/////
uint64_t EasyRF::convert_address(uint16_t addr) {
return (addr | (base_address & ~0xFFFFLL));
}
/////////////
bool EasyRF::RFSend(uint16_t to,const void* buf, uint8_t len){
    bool OK ;
    
 radio.stopListening(); 
 //if (rfSpeed == RF24_250KBPS) radio.flush_tx();
 radio.openWritingPipe(convert_address(to));
 OK = radio.write(buf,len); 
 radio.startListening(); 
 if (!autoACK) OK = true; 
 return OK; 
}
/////////////
bool EasyRF::RFMulticast(uint16_t to,const void* buf, uint8_t len){
    bool OK;
 radio.stopListening(); 
 if (rfSpeed == RF24_250KBPS) radio.flush_tx();

 radio.enableDynamicAck();
 radio.openWritingPipe(convert_address((uint16_t)to));
 OK = radio.write(buf,len,1); //write multicast, no ack.
 radio.startListening(); 
 if (!autoACK) OK = true; 

 return OK;   
}
/////////////////
uint8_t EasyRF::RFRead(void* buf){
 uint8_t len;
 if (dynPayload_en) len = radio.getDynamicPayloadSize();
 else len = max_payload;

 radio.read(buf,len); 
 
return len; 
 }
///////////////////////////
 void EasyRF::RFRead(void* buf,uint8_t byteLen){
 uint8_t len;
len = byteLen;

 radio.read(buf,len); 
 
//return len; 
 }
 ///////////////////////////
bool EasyRF::RFDataCome(){
 return radio.available();
}
///////////////////////////////
bool EasyRF::RFDataCome(uint8_t  &pipe){
 uint8_t pipeNo; 
 if (radio.available(&pipeNo)) {
 pipe = pipeNo;
  return true; 
 } 
 else 
  {
  return false; 
  }
}
/////////////////////////////////
uint8_t EasyRF::RFMultiCome(){
 uint8_t channel; 
 if (radio.available(&channel)==false) return 0; 
 else return channel; 
}
////////////////////////////////////////////////////////////////////////////////////////////////////