#include "nRF24Dongle.h"

void nRFDongle::init(){
Serial.begin(115200);
SPI.begin();
radio.begin();
network.begin(108,masterNode);// Channel Frequency = 108, Node add = 0 (master node)
   #ifdef DEBUG 
         Serial.print("Dongle begin with address: ");
         Serial.println(masterNode);
         Serial.print("State now: ");
         Serial.println(State);
         Serial.println("Go to Read Serial");
   #endif
}
///////////////////////////////////////
void nRFDongle::set_address(uint16_t nodeAddr){
	_slaveNode = nodeAddr; 
}
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
      Serial.print("*");
      #endif 
      //buffer[index]=c;
    }
    }else{
      prevc = c;  
      if(isStart){
        if(index==2){
         dataLen = c; 
          #ifdef DEBUG_SERIAL
         Serial.print(c);
         #endif 
         payloadLen = dataLen; 
        }else if(index>2){
          dataLen--;
            #ifdef DEBUG_SERIAL
             Serial.print(dataLen);
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
         for (int i=0;i<payloadLen+2;i++) {
           Serial.print(buffer[i]); Serial.print("-");
         }
         Serial.println();
         Serial.println("Goto Parsing");
         #endif
        index=0;
     }
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
        set_address(buffer[6]);        
        callOK();
         State = SERIAL_CHECK;         //Done, go back to Serial read for next message
                  first_run = true;      //set first run for next State

         }
        else { 
          State = RF_WRITE;  
                   first_run = true;      //set first run for next State

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
network.update();  
RF24NetworkHeader Writeheader(_slaveNode,'T'); //marking data stream is PC/Robot
  #ifdef DEBUG 
         Serial.print("..Sending data to address: ");
         Serial.println(_slaveNode);
   #endif
bool OK = network.write(Writeheader,buffer,payloadLen+2);
  #ifdef DEBUG 
         Serial.print("Sent!.. ");
   #endif
if (OK) {
  State = RF_READ;  //if onnect and send successfully 
   first_run = true;      //set first run for next State

   #ifdef DEBUG 
         Serial.print("Send Successfully to address: ");
         Serial.println(_slaveNode);
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
         Serial.println(_slaveNode);
        Serial.println("Go to back to read Serial");
     #endif 
   }
 }
}
///////////////////////////////////////////////////////////
void nRFDongle::readRF(){
network.update(); 
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
while ( network.available() )  {
     #ifdef DEBUG 
       Serial.println("RF data comming, read available");
     #endif
  RF24NetworkHeader Readheader;
  network.peek(Readheader);
  if(Readheader.type == 'T'){
    RFread_size = network.read(Readheader,RFbuf,MAX_READ_SIZE);
       #ifdef DEBUG 
         Serial.print("Read RF buffer from Slave Node address ");
         Serial.println(Readheader.from_node);
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
}
///////////////////////////////////
void nRFDongle::sendSerial(){
for (int i = 0;i<RFread_size;i++) { 
  Serial.print(RFbuf[i]);
  }
State = SERIAL_CHECK;  
first_run = true;      //set first run for next State

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
    
  case SETTING_ADDRESS :{
  sendSerial();
  }
  break; 
  } 
}