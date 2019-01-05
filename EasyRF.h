#ifndef EasyRF_H
#define EasyRF_H
/*
Library for RF24 application easy deployment for beginners. 
Source: Base on RF24 from Tmrh20 library
Author: Hien Phan
Company: Negendo
Date: Sep26,2018
*/
#include "RF24.h"
#ifndef DEBUG
//#define DEBUG
#endif
#define RFCHANNEL 108

#define TEMPLATE_ADDR 0xF0F0F00000LL 
#define MULTICAST_BASE 255

///
class RF24;
class EasyRF  {
public:
//EasyRF (int CE_PIN,int CSN_PIN): _CE(CE_PIN),_CSN(CSN_PIN) {}
//RF24 radio = RF24((uint16_t)_CE,(uint16_t)_CSN);
EasyRF(RF24& _radio);
RF24& radio; 
bool RFbegin(){
	return radio.begin();
}
bool init(uint16_t myaddress);
void RFpowerDown();
void RFpowerUp();
void setDynamicPayload(bool en) {
	dynPayload_en = en;
	//if (radio.isChipConnected()) {
	//	if (en)  radio.enableDynamicPayloads();
   // 	}

} 
void setMaxPayload(uint8_t max) {
	max_payload = max; 
}

void setDataSpeed(rf24_datarate_e speed) {
	if (radio.isChipConnected()) radio.setDataRate(speed);
	rfSpeed = speed;
}

void setPowerRF(rf24_pa_dbm_e pw){
	if (radio.isChipConnected()) radio.setPALevel(pw);
	rfPower = pw;
}

void setChannelRF(uint8_t ch) {
	if (radio.isChipConnected()) radio.setChannel(ch);
	myChannel = ch;
}

void setAutoACK(bool active) {
if (radio.isChipConnected()) radio.setAutoAck(active);
autoACK = active; 
}

void setRetry(int delay,int times) {
if (radio.isChipConnected()) radio.setRetries(delay,times);	
retryDelay = delay;
retryTimes = times;
}
////////////////////////////////////////////
bool checkCarrier() {					////
		return radio.testRPD();			
}										////
////////////////////////////////////////////
bool disableCRC(){
if (radio.isChipConnected()) {
	if  (!autoACK) {
		 radio.disableCRC();
		 return 1;
  		} 
		  else return 0;  // can not disable due to AutoACK is activated
	} else return 0;
}
//void init(uint16_t myaddress,uint8_t channel);

void SetAddress(uint16_t myaddress);      //pipe 1    USB dongle pairing 
void SetMultiCastAddress(uint16_t addr,uint8_t ch);//setup Multicast Address and Channel
bool RFSend(uint16_t to,const void* buf, uint8_t len);  //send a point to point Message
bool RFMulticast(uint16_t to,const void* buf, uint8_t len);  //Send a multicast Message to a Multicast Address
void Multicast_readingStart();
uint8_t RFRead(void* buf);
void RFRead(void* buf,uint8_t byteLen);
void RFRead_Multicast(void* buf); 
bool RFDataCome();       //check if RF data comming from channel 1 and ready for receive, get the comming data len
bool RFDataCome(uint8_t &pipe);

uint8_t RFMultiCome();  // check if RF Data comming from multi Channel, return the channel  (channel = 1-5)
//uint8_t getPayload_len();
private:
int _CE;
int _CSN; 
uint64_t convert_address(uint16_t addr); 
uint64_t address; 
uint16_t my_node;
uint16_t to_node; 
uint16_t multiCast_node = 250; 
uint8_t  multiCast_channel = 2; 
uint8_t myChannel = RFCHANNEL; 
uint8_t payload_len; 
uint8_t Mpayload_len;
uint64_t base_address = TEMPLATE_ADDR;
uint8_t max_payload = 32; 
bool  dynPayload_en = true; 
bool isRF24Connected = false;
rf24_crclength_e crcLen = RF24_CRC_8;
rf24_datarate_e rfSpeed = RF24_250KBPS;
rf24_pa_dbm_e rfPower = RF24_PA_LOW;
bool autoACK = true; 
int retryDelay = 5;
int retryTimes = 1;
};
#endif