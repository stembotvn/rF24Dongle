#include "nRF24Dongle.h"

void nRFDongle::init(){
Serial.begin(115200);
SPI.begin();
loadConfig();
radio.init(myNode);//init RF and setting Master Node address 
   #ifdef DEBUG 
         Serial.print("Dongle begin with address: ");
         Serial.println(myNode);
         Serial.print("State now: ");
         Serial.println(State);
         Serial.println("Go to Read Serial");
   #endif
pinMode(KEY,INPUT_PULLUP);  
//randomSeed(analogRead(0));

}
///////////////////////////////////////
void nRFDongle::set_address(uint16_t from,uint16_t to){
	myNode = from; 
  toNode = to;
  radio.init(myNode);
}
///////////////////////////////////////

////////////////////////////////////////
void nRFDongle::readSerial(){
isAvailable = false; 
if (Serial.available() > 0){
  isAvailable = true; 
  serialRead = Serial.read();
  }
if (isAvailable) {
    #ifdef DEBUG_SERIAL
    Serial.print(".");
    #endif
   unsigned char c = serialRead&0xff;
    if(c==0x55&&isStart==false){
     if(prevc==0xff){
      index=1;
      isStart = true;
      #ifdef DEBUG_SERIAL
     // Serial.print("*");
      #endif 
      //buffer[index]=c;
    }
    }else{
      prevc = c;  
      if(isStart){
        if(index==2){
         dataLen = c; 
          #ifdef DEBUG_SERIAL
         Serial.print(c,HEX);
         Serial.print(" ");
         #endif 
         payloadLen = dataLen; 
        }else if(index>2){
          dataLen--;
            #ifdef DEBUG_SERIAL
             Serial.print(dataLen);
             Serial.print(" ");
            #endif
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
         first_run = true;      //set first run for next State
         #ifdef DEBUG 
         Serial.print("Valid Data coming, number of payload bytes: ");Serial.println(payloadLen);
         for (int i=0;i<payloadLen+3;i++) {
           Serial.print(buffer[i],HEX); Serial.print("-");
         }
         Serial.println();
         Serial.println("Goto Parsing");
         #endif
        index=0;
     }
  }
 
   if (!isStart&&!isAvailable) 
     {
        checkConfig();
    }
  
}
///////////////////////////////////////////////////////////////////
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
    case  GET: { 
       State  = RF_WRITE; 
       first_run = true;      //set first run for next State

       timeStart = millis();
      timeout=GET_TIMEOUT;
       }
    break;
     case RUN:{
       if (action_type == SET_ADDRESS) { // setting target address for Robot; 
        set_address(MASTER,buffer[6]);      //save new address to RAM for next CONFIG PROCESS  
        callOK();
        if (buffer[6]==255) mode = MULTICAST;
        else mode = UNICAST;
        configMode = NETWORK_ADDRESSING;
        State = SERIAL_CHECK;         //Done, go back to Serial read for next message
        first_run = true;      //set first run for next State
         }
         else { 
          State = RF_WRITE;  
          first_run = true;     //set first run for next State
          timeStart = millis();    
           #ifdef DEBUG 
           Serial.println("Goto Send RF");
           #endif
        }
        timeout=RUN_TIMEOUT; 
     }
      break;
      case RESET:{
        //reset
        callOK();
        State = SERIAL_CHECK; 
        first_run = true;      //set first run for next State

      }
     break;
     case START:{
        //start
        callOK();
        State = SERIAL_CHECK; 
        first_run = true;      //set first run for next State

      }
     break;
     
  }
}

///////////////////////////////////////////////////////////////
void nRFDongle::writeRF(){
  #ifdef DEBUG 
         Serial.print("..Sending data to address: ");
         Serial.println(toNode);
   #endif

bool OK = radio.RFSend(toNode,buffer,payloadLen+3);
  #ifdef DEBUG 
         Serial.print("Sent!.. ");
   #endif
if (OK) {
  State = RF_READ;  //if onnect and send successfully 
   first_run = true;      //set first run for next State

   #ifdef DEBUG 
         Serial.print("Send Successfully to address: ");
         Serial.println(toNode);
         Serial.println("Go to Read RF");
   #endif
}
else {
   if (millis()-timeStart>timeout/2) {
   callOK();    
   State = SERIAL_CHECK;    //exit when time out
            first_run = true;      //set first run for next State
     #ifdef DEBUG 
         Serial.print("Sending fail to address: ");
         Serial.println(toNode);
        Serial.println("Go to back to read Serial");
     #endif 
   }
 }
}
///////////////////////////////////////////////////////////
void nRFDongle::readRF(){
RFread_size = 0;
if (millis()-timeStart >timeout) {
     #ifdef DEBUG 
         Serial.print("..Time out, not received response");
         Serial.println("Go back to Read Serial");
     #endif
  callOK();
  State = SERIAL_CHECK; 
           first_run = true;      //set first run for next State
  
  return;
}
if ( radio.RFDataCome() )  {
     #ifdef DEBUG 
       Serial.println("RF data comming, read available");
     #endif
    RFread_size = radio.RFRead(RFbuf);
       #ifdef DEBUG 
         Serial.print("Read RF buffer from Slave Node address ");
         Serial.println(toNode);
         for (int i = 0;i<RFread_size;i++) {
           Serial.print(RFbuf[i],HEX);Serial.print(" ");
         }
         Serial.println();
         Serial.println("Go to Write Serial data:");
       #endif
    }
  if (RFread_size > 1) {
     State = SERIAL_SEND;
              first_run = true;      //set first run for next State

        }
  else {
   callOK(); 
      #ifdef DEBUG 
         
         Serial.println("Data received not match");
   #endif   
   State = SERIAL_CHECK; 
   first_run = true;      //set first run for next State

  } 
 }

///////////////////////////////////
void nRFDongle::sendSerial(){
#if DEBUG
Serial.print("SENDING DATA RESPONSE TO PC...:"); Serial.println(RFread_size);
#endif
for (int i = 0;i<RFread_size;i++) { 
  Serial.write(RFbuf[i]);
  _delay_us(100);
  //Serial.print(i);
  }
State = SERIAL_CHECK;  
first_run = true;   //set first run for next State
#if DEBUG
Serial.println("SENDING DONE! BACK TO READ SERIAL COMMAND");
#endif
}
///////////////////////////////////
void nRFDongle::run(){
if (first_run)  {
   timeStart = millis();
   #ifdef DEBUG 
         Serial.print("State No: ");
         Serial.println(State);
         Serial.println(" Begin");
   #endif
   first_run = false; 
}  
switch (State) {

  case SERIAL_CHECK :{
  readSerial(); 
  }
  break; 

  case SERIAL_PARSING :{
  parsingSerial();
  }
  break;

  case RF_WRITE :{
  writeRF();
  }
  break;
 
  case RF_READ :{
  readRF();
  }
  break; 

  case SERIAL_SEND :{
  sendSerial();
  }
  break; 
    
 
  } 
}
///////////////////////////
///Private function////////
void nRFDongle::checkConfig() {
bool accessed = false;
if (!digitalRead(KEY)) {
  double start = millis();  
  while (!digitalRead(KEY)) {
    if (millis()-start>500) accessed = true; 
  }
  if (accessed) {//access Sending CONFIG IF PRESS AND HOLD KEY IN 2 SEC
  // configMode = RANDOOM_ADDRESSING;
      #ifdef DEBUG
      Serial.println("Config mode is accessed");
      #endif
      if (configMode == RANDOOM_ADDRESSING) {
       #ifdef DEBUG
      Serial.println("RANDOM ADDRESSING MODE CONFIG");
      #endif
      randomSeed(millis());
      myNode = random(256,999);      //get randoom of my Address
      toNode = random(1001,2000);  //get randoom of Targeting Address
   //   myNode = (uint16_t)(millis()-start)/2;
   //   toNode = (uint16_t)(millis()-start);
      #ifdef DEBUG
      Serial.println("Got new address");
      Serial.print("My address: ");Serial.print(myNode,HEX);Serial.print("  Target address: ");Serial.println(toNode,HEX);
      #endif
      delay(1000);
      radio.init(myNode); // update my address
      mode = UNICAST; 
      sendConfig();
      saveConfig();
      accessed = false; 
      }
      else if (configMode == NETWORK_ADDRESSING) {
      #ifdef DEBUG
      Serial.println("RANDOM ADDRESSING MODE CONFIG");
      #endif  
      sendConfig();  //just send config to target, not save in DOngle. 
      }
   }
   else {
      #ifdef DEBUG
      Serial.println("Not access to CONFIG MODE");
      #endif 
   }
 }

}
/////
void nRFDongle::sendConfig(){
 #ifdef DEBUG
      Serial.println("Sending config data:...");
      Serial.print("USB Address: ");Serial.print(myNode); Serial.print("   Robot address: "); Serial.println(toNode);
      #endif
idx = 0;
CFGbuffer[idx++] = 0xFF;
CFGbuffer[idx++] = 0x55;
CFGbuffer[idx++] = 0x00;   //Len = 6 bytes
CFGbuffer[idx++] = 0x00;
CFGbuffer[idx++] = 0x02; //RUN, NOT GET RESPONSE VALUE
CFGbuffer[idx++] = 80;   // CONFIG ADDRESSING TYPE OF COMMAND
addValue(idx,toNode);
addValue(idx,myNode);
//CFGbuffer[idx] = 0xA; // line Feed
int len = idx + 1;
CFGbuffer[2] = len-3;
#ifdef DEBUG
  for (int i=0;i<len;i++) {
    Serial.print(CFGbuffer[i],HEX); Serial.print(" ");
  }
  Serial.println();
  #endif
bool OK=radio.RFSend(Default_Addr,CFGbuffer,len);
if (OK) { 
  #ifdef DEBUG
  Serial.println("Sent Config addressing successful ");
  #endif
    }
else {
  #ifdef DEBUG
  Serial.println("Sent Config addressing FAIL!");
  #endif
 }  
}
////////////////////////////////////////////////////
void nRFDongle::addValue(int pos,uint16_t val) {
idx = pos;  
valShort.shortVal = val; 
CFGbuffer[idx++] = valShort.byteVal[0];
CFGbuffer[idx++] = valShort.byteVal[1];
}
//////////////////////////////////////////////////
void nRFDongle::saveConfig(){
EEPROM_writeInt(0,myNode);
EEPROM_writeInt(2,toNode);

}
////
void nRFDongle::loadConfig(){
uint16_t myAd,toAd;  
myNode = EEPROM_readInt(0);
toNode = EEPROM_readInt(2);
//set_address(myAd,toAd);
}
//////////////////////////////////////////
void nRFDongle::EEPROM_writeInt(int address,uint16_t value) {
  
      //Decomposition from a int to 2 bytes by using bitshift.
      //One = Most significant -> Two = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);
      

      //Write the 2 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
     
     
}
/////////////////////////
uint16_t nRFDongle::EEPROM_readInt(int address){
uint16_t two = EEPROM.read(address);
uint16_t one = EEPROM.read(address+1); 
return ((two & 0xFF) + ((one<<8)&0xFFFF));
}