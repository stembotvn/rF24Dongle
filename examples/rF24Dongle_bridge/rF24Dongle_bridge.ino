#include <nRF24Dongle.h>
nRFDongle  dongle; 

void setup() {
dongle.init();

}

void loop(){
dongle.run();
}