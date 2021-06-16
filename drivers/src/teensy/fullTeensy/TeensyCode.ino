#include <Servo.h>
#include <sstream>
#include <vector>
#include <string>

#include <Wire.h>
#include "MS5837.h"

#include <SoftwareSerial.h>

int rxPin = 21;
int txPin = 20;
SoftwareSerial sSerial = SoftwareSerial(rxPin, txPin);

MS5837 sensor;

Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;

Servo* thrusters[] = {&thruster1, &thruster2, &thruster3, &thruster4, &thruster5, &thruster6, &thruster7, &thruster8};

String input = ",";

void setup() {
  // put your setup code here, to run once:

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  sSerial.begin(115200);
  Serial.begin(115200);

  Serial.println("Starting");

  Wire.begin();

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setFluidDensity(997);
  
  thruster1.attach(0);
  thruster2.attach(1);
  thruster3.attach(2);
  thruster4.attach(3);
  thruster5.attach(4);
  thruster6.attach(5);
  thruster7.attach(6);
  thruster8.attach(7);

  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);
  thruster3.writeMicroseconds(1500);
  thruster4.writeMicroseconds(1500);
  thruster5.writeMicroseconds(1500);
  thruster6.writeMicroseconds(1500);
  thruster7.writeMicroseconds(1500);
  thruster8.writeMicroseconds(1500);
  
}

void loop() {
  // put your main code here, to run repeatedly:

 sensor.read();

 Serial.print("depth,");
 Serial.println(sensor.depth());

 if (sSerial.available()) {
    String msg = sSerial.readStringUntil('\n');
    msg = msg.substring(3);
    msg = String("DVL") + msg;
    Serial.write(msg.c_str());
  }

 if (Serial.available() > 0) {
    Serial.println(Serial.available());
		input = Serial.readStringUntil('\n');
    if (input.indexOf(',') == -1) {
      input = ",";
    }
    
  }

  std::string instruction = std::string(input.c_str());

  std::stringstream ss(instruction);

  std::vector<String> result;

  while(ss.good()) {

    std::string substr;
    getline(ss, substr, ',');
    result.emplace_back(substr.c_str());
    
  }

  if (result[0] == "thrust") {

    for (unsigned int i = 1; i < result.size(); i++) {
    
      Servo* thruster = thrusters[result[i].substring(0, 1).toInt()];
  
      int val = result[i].substring(1).toInt();
      
      thruster->writeMicroseconds(val);

    }
      
  }

}
