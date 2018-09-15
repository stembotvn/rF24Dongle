//library for USD RF24 Dongle fimware for Scratch mBlock interface
//Hardware: AT328P  Arduino Nano Bootloader
//Serial to RF24 Bridge 
//Author: Hien PHan
//Project: Negendo Dongle for Scratch 
//Negendo Toys company 

#ifndef rFDongle_H
#define rFDongle_H

#include "RF24.h"
#include <SPI.h>
#include "Scratch.h"

/////Pins define 
#define CE_PIN    9
#define CSN_PIN   10
#define Button    3
#define LED       4
///Class define
class rFDongle {
public:


private: 
  RF24 radio = RF24(CE_PIN, CSN_PIN);
  const uint64_t _AddDefault = 0xF0F0F0F000LL;  // Địa chỉ truyền tín hiệu NRF24L01 mặc định



};


#endif 