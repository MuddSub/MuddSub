#include <SoftwareSerial.h>
#include <string>

int rxPin = 21;
int txPin = 20;
SoftwareSerial sSerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  sSerial.begin(115200);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sSerial.available()) {
    String msg = sSerial.readStringUntil('\n');
    msg = msg.substring(3);
    msg = String("DVL") + msg;
    Serial.write(msg.c_str());
  }
}
