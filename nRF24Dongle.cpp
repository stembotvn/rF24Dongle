#include "nRF24Dongle.h"

void nRFDongle::init(){
Serial.begin(115200);
SPI.begin();
loadConfig();
radio.init(myNode);//init RF and setting Master Node address 
   #ifdef DEBUG_CONFIG
         Serial.print("Dongle begin with address: ");
         Serial.print(myNode);
         Serial.print("         -Target address: ");
         Serial.println(toNode);
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
////
void nRFDongle::clearBuffer(unsigned char *buf, int leng){
  for (int i=0;i<leng;i++) {
    *(buf+i) = 0; 
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
    int idx = buffer[3];
  int action = buffer[4];
  int action_type = buffer[5];
  switch(action){
    case  GET: { 
      if (action_type!=DONE) {
       State  = RF_WRITE; 
       first_run = true;      //set first run for next State
       timeStart = millis();
      timeout=GET_TIMEOUT;
      }
      else {
          #ifdef DEBUG
          Serial.print("Check command done from robot..");
          #endif 
          Serial.write(0xff);
          delayMicroseconds(100);
          Serial.write(0x55);      
         delayMicroseconds(100);
            Serial.write(idx);
          delayMicroseconds(100);
          
          sendFloat(done);
          Serial.println();
            #ifdef DEBUG
          Serial.print("Result: ..");          Serial.print(done);
          Serial.println("   Go back to check Serial");

          #endif 
          State = SERIAL_CHECK;
          clearBuffer(buffer,32);

        first_run = true;
      }
       }
    break;
     case RUN:{
       if (action_type == SET_ADDRESS) { // setting target address for Robot; 
        callOK();
        if (buffer[6]==255) mode = MULTICAST;
        else if (buffer[6]==0) mode = UNICAST;
        State = SERIAL_CHECK;         //Done, go back to Serial read for next message
         first_run = true;      //set first run for next State
         clearBuffer(buffer,32);
         }
         else { 
          done = false;  
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
    default: {
       State = SERIAL_CHECK;
       clearBuffer(buffer,32);

        first_run = true;
    }
     
  }
}
////
void nRFDongle::clearRX(){
  if (radio.RFDataCome()) {
    while (radio.RFDataCome()) radio.RFRead(dump);
  }
}
///////////////////////////////////////////////////////////////
void nRFDongle::writeRF(){
  bool OK;
  
//clearRX();
  if(mode == UNICAST){
     OK = radio.RFSend(toNode,buffer,payloadLen+3);
     #ifdef DEBUG 
         Serial.print("..Sending data to address: ");
         Serial.println(toNode);
    #endif
    #ifdef DEBUG 
         Serial.println("  Sent!.. ");
    #endif
  }
  else{
    #ifdef DEBUG 
         Serial.println("..Sending data to Multicast");
    #endif
    OK = radio.RFMulticast(250,buffer,payloadLen+3);
    
  }
if (OK) {
  if(mode == UNICAST){
   State = RF_READ;  //if onnect and send successfully 
   first_run = true;      //set first run for next State
   timeStart = millis();

   #ifdef DEBUG 
         Serial.print("Send Successfully to address: ");
         Serial.println(toNode);
         Serial.println("Go to Read RF");
   #endif
   clearBuffer(buffer,32);
   clearBuffer(RFbuf,32);
  }
  else {  //in Multicast mode 
     #ifdef DEBUG 
         Serial.print("Send Successfully to Multicast Channel ");
         Serial.println("- Go to Serial Check");
   #endif
    State = SERIAL_CHECK;  //if onnect and send successfully 
    first_run = true;      //set first run for next State
    clearBuffer(buffer,32);
    clearBuffer(RFbuf,32);
  }
   return;
}
else {
  // if (millis()-timeStart>timeout/2) {
   callOK();    
   State = SERIAL_CHECK;    //exit when time out
            first_run = true;      //set first run for next State
            done = true;
   clearBuffer(buffer,32);
    clearBuffer(RFbuf,32);   
    init();
     #ifdef DEBUG 
         Serial.print("Sending fail to address: ");
         Serial.println(toNode);
        Serial.println("Go to back to read Serial");
     #endif 
    return; 
  // }
 }
}
///////////////////////////////////////////////////////////
void nRFDongle::readRF(){
RFread_size = 0;
clearBuffer(RFbuf,32);
if (millis()-timeStart >timeout) {  //if no data come in over timeout, return
    
     #ifdef DEBUG 
         Serial.print("..Time out, not received response");
         Serial.println("Go back to Read Serial");
     #endif
     Serial.println("READING TIMEOUT");
  callOK();
  State = SERIAL_CHECK; 
           first_run = true;      //set first run for next State
           done = true;          //if timeout and not received, skip command
           clearBuffer(buffer,32);
           init();
  
  return;
}
if ( radio.RFDataCome() )  {
     #ifdef DEBUG 
       Serial.println("RF data comming, read available");
     #endif
     while (radio.RFDataCome() )  RFread_size = radio.RFRead(RFbuf);
       #ifdef DEBUG 
         Serial.print("Read RF buffer from Slave Node address ");
         Serial.println(toNode);
         
         for (int i = 0;i<RFread_size;i++) {
           Serial.print(RFbuf[i],HEX);Serial.print(" ");
         }
         Serial.println();
         Serial.println("Go to Write Serial data:");
       #endif
   // }
  if (RFread_size >=4) {
     done = true; 
     State = SERIAL_SEND;
     first_run = true;      //set first run for next State
     if (RFbuf[0]==0xFF && RFbuf[1]==0x55 && RFbuf[2]==0xD && RFbuf[3]==0xA) done = true; //done signal from robot
        }
  else {
   callOK(); 
      #ifdef DEBUG 
         
         Serial.println("Data received not match");
   #endif   
   State = SERIAL_CHECK; 
   clearBuffer(buffer,32);
   first_run = true;      //set first run for next State
   done = true; //skip command 
    } 
  }
 }

////////////////////////////////////////////////////////////////////////////////
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
clearBuffer(buffer,32);
#if DEBUG
Serial.println("SENDING DONE! BACK TO READ SERIAL COMMAND");
#endif
}
//////////////////////////////////////////////////////////////////////////////////
void nRFDongle::run(){
if (first_run)  {
   timeStart = millis();
   #ifdef DEBUG_STATE 
         Serial.print("State No: ");
         Serial.println(State);
       //  Serial.println(" Begin");
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
int new_myNode;
int new_toNode;
if (!digitalRead(KEY)) {
  double start = millis();  
  while (!digitalRead(KEY)&&!accessed) {
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
      new_myNode = random(1000,10000);      //get randoom of my Address
      new_toNode = random(20000,50000);  //get randoom of Targeting Address
   //   myNode = (uint16_t)(millis()-start)/2;
   //   toNode = (uint16_t)(millis()-start);
      #ifdef DEBUG
      Serial.println("Got new address");
      Serial.print("My address: ");Serial.print(new_myNode,HEX);Serial.print("  Target address: ");Serial.println(new_toNode,HEX);
      #endif
      delay(1000);
      
      mode = UNICAST; 
      if (sendConfig(new_myNode,new_toNode)) {
      saveConfig(new_myNode,new_toNode);
      myNode = new_myNode;   //
      toNode = new_toNode;
      radio.init(myNode); // update my address
      #ifdef DEBUG
      Serial.println("Set new address successfully");
      #endif
      } 
      else {
      #ifdef DEBUG
      Serial.println("Set new address fail, please check the robot connection");
      #endif  
      }
       
      accessed = false; 
      }
      else if (configMode == NETWORK_ADDRESSING) {
      #ifdef DEBUG
      Serial.println("RANDOM ADDRESSING MODE CONFIG");
      #endif  
      }
    //  sendConfig();  //just send config to target, not save in DOngle. 
      }
   
   else {
      #ifdef DEBUG
      Serial.println("Not access to CONFIG MODE");
      #endif 
   }
 }

}
/////
bool nRFDongle::sendConfig(int _to,int _my){
 #ifdef DEBUG
      Serial.println("Sending config data:...");
      Serial.print("USB Address: ");Serial.print(_my); Serial.print("   Robot address: "); Serial.println(_to);
      #endif
idx = 0;
CFGbuffer[idx++] = 0xFF;
CFGbuffer[idx++] = 0x55;
CFGbuffer[idx++] = 0x00;   //Len = 6 bytes
CFGbuffer[idx++] = 0x00;
CFGbuffer[idx++] = 0x02; //RUN, NOT GET RESPONSE VALUE
CFGbuffer[idx++] = 80;   // CONFIG ADDRESSING TYPE OF COMMAND
addValue(idx,_to);
addValue(idx,_my);
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
   return OK;

}
////////////////////////////////////////////////////
void nRFDongle::addValue(int pos,uint16_t val) {
idx = pos;  
valShort.shortVal = val; 
CFGbuffer[idx++] = valShort.byteVal[0];
CFGbuffer[idx++] = valShort.byteVal[1];
}
//////////////////////////////////////////////////
void nRFDongle::saveConfig(int new_my,int new_to){
EEPROM_writeInt(0,new_my);
EEPROM_writeInt(2,new_to);

}
////
void nRFDongle::loadConfig(){
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
////
void nRFDongle::sendFloat(float value){ 
     Serial.write(0x2);
               delayMicroseconds(100);

     val.floatVal = value;
     Serial.write(val.byteVal[0]);
    delayMicroseconds(100);
     Serial.write(val.byteVal[1]);
      delayMicroseconds(100);

     Serial.write(val.byteVal[2]);
      delayMicroseconds(100);

     Serial.write(val.byteVal[3]);
     delayMicroseconds(100);

}