//library for USD RF24 Dongle fimware for Scratch mBlock interface
//Hardware: AT328P  Arduino Nano Bootloader
//Serial to Nordic RF24L01 Bridge for mBlock software
//Author: Hien PHan
//Project: Negendo Dongle for Scratch 
//Negendo Toys company 

#ifndef rFDongle_H
#define rFDongle_H
//#define DEBUG 1
//#define DEBUG_SERIAL 1////////////////////////////////////
#include "EasyRF.h"
#include <SPI.h>
#include "Scratch.h"
#include "EEPROM.h"
/////Pins define///////////////////////////////////////////
#define CE_PIN  9
#define CSN_PIN 10
#define KEY     3
#define LED     4
/////State define//////////////////////////////////////////
#define SERIAL_CHECK     0     //check data from PC     ///
#define SERIAL_PARSING   1     //Parsing data received from PC
#define RF_WRITE         2     //Send data via RF       
#define RF_READ          3     //Reading data from RF
#define SERIAL_SEND      4     //send data to PC
#define COMMAND_DONE     5     
#define SETTING_ADDRESS  6
#define CONFIG           7
///////////////////////////////////////////////////////////
//define Serial command mode//////////////////////////////
/////////////////////////////////////////////////////////////
#define GET 1
#define RUN 2
#define RESET 4
#define START 5
//////
#define MASTER    0 //address for Master
/////Mode 
#define MULTICAST     1
#define UNICAST       0
////
#define MULTICAST_ADDRESS 255
#define MULTICAST_CHANNEL 2 
#define DEFAULT_ADDRESS 1000
/////
#define MAX_READ_SIZE 32
#define RUN_TIMEOUT    10000L  
#define GET_TIMEOUT    10000L
///define for New 
#define RANDOOM_ADDRESSING 0
#define NETWORK_ADDRESSING 1
////
class nRFDongle {
public:
nRFDongle() {} //constructor//////////////////////////////// 
int State = SERIAL_CHECK; 
RF24 myRadio = RF24(CE_PIN, CSN_PIN);
EasyRF radio = EasyRF(myRadio);   
void init(); 
void set_address(uint16_t from,uint16_t to);
void readSerial();
void parsingSerial();
void writeRF();
void readRF();
void callOK();
void sendSerial();
void checkConfig();
void sendConfig();
void run();

private: 
uint16_t myNode = MASTER; 
uint16_t toNode = 2;    
uint16_t multiCast_Node = MULTICAST_ADDRESS;
uint16_t  Default_Addr = DEFAULT_ADDRESS;
uint8_t configMode = RANDOOM_ADDRESSING; 
bool mode = UNICAST;            ///send to 1 node 
uint8_t payloadLen;             
////variable for Serial function
bool isAvailable = false; 
bool isStart = false;
unsigned char prevc = 0; 
byte index = 0;
byte dataLen = 0;
unsigned char buffer[32]; // buffer for serial read data 
unsigned char RFbuf[32]; 
int RFread_size=0; 
unsigned char serialRead;
double timeStart; 
unsigned long timeout = RUN_TIMEOUT; 
bool first_run = true; 
bool isGetNewAddress = false; 
byte idx = 0;
uint8_t CFGbuffer[32]; 
////////Define  RF Scratch command array processing////////////
   union
  {
    byte byteVal[2];
    short shortVal;
  }valShort;
  //////////////////////////////////////////////////////////////
  void startPackage(uint8_t *buf,uint8_t type,uint8_t action); 
  void addValue(int pos,uint16_t value);
  void writeHead();
  void writeEnd();
  /////////////////////////////////////////
  void writeBuffer(int index,unsigned char c); //write to RF Sending Buffer
  /////////////////////////////////
  void sendShort(double value); 
  void saveConfig();
  void loadConfig();
  void EEPROM_writeInt(int address,uint16_t value);
  uint16_t EEPROM_readInt(int address);

};/////
#endif
