#include <rFDongle.h>
rFDongle  dongle; 

void setup() {
dongle.init();

}

void loop(){
dongle.run();
}