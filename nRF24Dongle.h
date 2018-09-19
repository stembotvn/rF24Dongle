//library for USD RF24 Dongle fimware for Scratch mBlock interface
//Hardware: AT328P  Arduino Nano Bootloader
//Serial to RF24 Bridge 
//Author: Hien PHan
//Project: Negendo Dongle for Scratch 
//Negendo Toys company 

#ifndef rFDongle_H
#define rFDongle_H

#include "RF24.h"
#include "RF24Network.h"
#include <SPI.h>
#include "Scratch.h"

/////Pins define 
#define CE_PIN    9
#define CSN_PIN   10
#define Button    3
#define LED       4
/////State define
#define SerialCheck     0     //check data from PC
#define RFwrite         1     //Send data via RF 
#define RFread          2     //Reading data from RF
#define RFreadParsing   3     //Parsing data received from RF
#define SerialSend      4     //send data to PC
#define Ending          5     //ending the transceving 
#define networkUpdate   6     //update network Status 
//////
#define MasterNode    0 
#define Multicast     1
#define Unicast       0
///Class define
class nRFDongle {
public:
nRFDongle() {} //constructor 
int State = SerialCheck; 

RF24 radio = RF24(CE_PIN, CSN_PIN);
RF24Network network = RF24Network(radio);    
void init(); 
void set_address(uint16_t nodeAddr);
void readSerial();
void writeRF();
void readRF();
void parsingRFread();
void sendSerial();
void endProcess();
void run();

private: 

uint16_t _slaveNode = 02; 
uint16_t _multicastLevel = 01; 
bool mode = Unicast;            ///send to 1 node 
};


#endif 