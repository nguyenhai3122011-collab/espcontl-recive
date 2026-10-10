#include "ReceiverLink.h"

ReceiverLink receiver;
ctl::Packet latest;
uint32_t lastPrint=0;
bool outputActive=false,newData=false;

void stopOutputs() {
  bool wasActive=outputActive;
  latest=ctl::Packet{}; outputActive=false; newData=false;
  // REQUIRED for real hardware: stop PWM, disable drivers, relays, etc. here.
  // This demo never energizes a GPIO.
  if(wasActive) Serial.println("STOP: neutral / control timeout"); // one transition only
}
void onControl(const ctl::Packet &p) {
  if(!outputActive) Serial.println("CONNECTED: receiving control");
  latest=p; outputActive=true; newData=true;
  // Apply validated p.axis[] / p.buttons HERE. Never put delay() in this handler.
}
void setup() {
  Serial.begin(115200);
  if(!receiver.begin("espH_TestReceiver",ctl::ROLE_RECEIVER,onControl,stopOutputs)) {
    Serial.println("ERROR: receiver init failed"); while(true) delay(1000);
  }
  // No startup banner and no fake zero-value stream while waiting for a controller.
}
void loop() {
  receiver.update();
  if(receiver.active() && outputActive && newData && millis()-lastPrint>=200) {
    lastPrint=millis(); newData=false;
    Serial.printf("RX seq=%lu J1=(%d,%d) J2=(%d,%d) L=%u R=%u SW1=%u SW2=%u\n",
      (unsigned long)latest.seq,latest.axis[0],latest.axis[1],latest.axis[2],latest.axis[3],
      !!(latest.buttons&ctl::B_L),!!(latest.buttons&ctl::B_R),
      !!(latest.buttons&ctl::B_J1),!!(latest.buttons&ctl::B_J2));
  }
  delay(1);
}
