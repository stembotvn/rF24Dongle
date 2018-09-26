//library for USD RF24 Dongle fimware for Scratch mBlock interface
//Hardware: AT328P  Arduino Nano Bootloader
//Serial to Nordic RF24L01 Bridge for mBlock software
//Author: Hien PHan
//Project: Negendo Dongle for Scratch 
//Negendo Toys company 

#ifndef rFDongle_H
#define rFDongle_H
#define DEBUG 1
//#define DEBUG_SERIAL 1
#include "RF24.h"
#include "RF24Network.h"
#include <SPI.h>
#include "Scratch.h"

/////Pins define////////////////////////////////////////
#define CE_PIN    9
#define CSN_PIN   10
#define Button    3
#define LED       4
/////State define
#define SERIAL_CHECK     0     //check data from PC
#define SERIAL_PARSING   1     //Parsing data received from PC
#define RF_WRITE         2     //Send data via RF 
#define RF_READ          3     //Reading data from RF
#define SERIAL_SEND      4     //send data to PC
#define COMMAND_DONE     5 
#define SETTING_ADDRESS  6
//////
//define Serial command mode//////////////////////////////
#define GET 1
#define RUN 2
#define RESET 4
#define START 5
//////
#define MASTER    0 //address for Master
#define Multicast     1
#define Unicast       0
#define MAX_READ_SIZE 32
#define RUN_TIMEOUT    10000L  
#define GET_TIMEOUT    10000L
///Class define

class nRFDongle {
public:
nRFDongle() {} //constructor//////////////////////////////// 
int State = SERIAL_CHECK; 
RF24 radio = RF24(CE_PIN, CSN_PIN);
RF24Network network = RF24Network(radio);    
void init(); 
void set_address(uint16_t nodeAddr);
void readSerial();
void parsingSerial();
void writeRF();
void readRF();
void callOK();
void sendSerial();
void run();

private: 
uint16_t masterNode = MASTER; 
uint16_t _slaveNode = 02; 
uint16_t _multicastLevel = 01; 
bool mode = Unicast;            ///send to 1 node 
uint8_t payloadLen; 
////variable for Serial function
bool isAvailable = false; 
bool isStart = false;
unsigned char prevc = 0; 
byte index = 0;
byte dataLen = 0;
unsigned char buffer[32]; // buffer for serial read data
unsigned char RFbuf[32]; 
int idx = 0;
int RFread_size=0; 
unsigned char serialRead;
double timeStart; 
unsigned long timeout = RUN_TIMEOUT; 
bool first_run = true; 
};/////
#endif 