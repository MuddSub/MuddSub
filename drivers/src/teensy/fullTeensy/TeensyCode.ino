/* This code is for the 2023 RoboSub Competition.
    Features:
      - Taking in serial commands from ROS, sending them to thrusters
      - Sending depth sensor readings over serial to ROS
      - Sending a serial message when a pin is asserted. This is to start the mission using the 'on'
        switch.
*/


#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

#define RX_PIN 21
#define TX_PIN 20

// on switch is active low
#define ON_SWITCH_PIN 33

// delay period in ms
#define DELAY_PERIOD 30

// memory buffer size for input serial commands
#define RX_BUF_SIZE 128

// experimental max and min PWM values for these thrusters
// the max PWM the thrusters responded to was 2200, but this is set
// to 2100 for symmetry with the 900 lower bound
#define MIN_PWM 900
#define MAX_PWM 2100

MS5837 depth_sensor;

Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;

Servo *thrusters[] = {&thruster1, &thruster2, &thruster3, &thruster4, &thruster5, &thruster6, &thruster7, &thruster8};

// String input = ",";
char serial_cmd_input[RX_BUF_SIZE];

float fluid_density, pressure_offset;

/* runs when interrupt pin changes */
void send_start_msg_isr()
{
  if(digitalRead(ON_SWITCH_PIN)) 
  {
    Serial.printf("switch,off\n");
  }
  else 
  {
    Serial.printf("switch,on\n");
  }
  return;
}

void setup()
{
  // put your setup code here, to run once:

  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(ON_SWITCH_PIN, INPUT_PULLUP);

  // enable interrupt for pin ON_SWITCH_PIN
  attachInterrupt(digitalPinToInterrupt(ON_SWITCH_PIN), send_start_msg_isr, CHANGE);

  Serial.begin(115200);

  Serial.println("Starting");

  Wire.begin();

  thruster1.attach(0);
  thruster2.attach(1);
  thruster3.attach(2);
  thruster4.attach(3);
  thruster5.attach(4);
  thruster6.attach(5);
  thruster7.attach(6);
  thruster8.attach(7);

  // set thrusters to 1500, which corresponds to motors off
  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);
  thruster3.writeMicroseconds(1500);
  thruster4.writeMicroseconds(1500);
  thruster5.writeMicroseconds(1500);
  thruster6.writeMicroseconds(1500);
  thruster7.writeMicroseconds(1500);
  thruster8.writeMicroseconds(1500);

  while (!depth_sensor.begin(Wire))
  {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  fluid_density = 997; // kg/m^3
  depth_sensor.setFluidDensity(fluid_density);
  depth_sensor.setModel(MS5837::MS5837_30BA);

  delay(500);

  // Get atmospheric pressure to prevent hard-coding offset
  // This takes the first reading, assuming the Teensy is first initally powered outside of water
  depth_sensor.read();
  pressure_offset = depth_sensor.pressure(MS5837::Pa); // offset in Pa

  Serial.printf("Pressure offset is %f\r\n", pressure_offset);

  // If Teensy is started at a pressure above air pressure bounds, set it to standard air pressure
  if (pressure_offset > 108400 || pressure_offset < 87000) // 108400 Pa is the highest recorded air pressure.
  {
    // use standard average air pressure
    pressure_offset = 101300;
  }
  
}

void loop()
{
  // put your main code here, to run repeatedly:

  depth_sensor.read();

  // Calculate depth given pressure
  // depth is in meters
  float depth = (depth_sensor.pressure(MS5837::Pa) - pressure_offset) / (fluid_density * 9.80665);

  Serial.printf("depth,%f\n", depth);

  int input_len = 0;
  // receive incoming bytes from serial
  while (Serial.available() && input_len < (RX_BUF_SIZE - 1))
  {
    // put incoming byte into serial_cmd_input char array
    serial_cmd_input[input_len] = Serial.read();
    input_len++;
  }
  serial_cmd_input[input_len] = '\0';

  if (input_len > 0) 
  {
    int pwms[8];
    // listen to a serial command of the following format, put the values in the pwms array
    int result = sscanf(serial_cmd_input, "thrust,0%d,1%d,2%d,3%d,4%d,5%d,6%d,7%d", 
        &pwms[0], &pwms[1], &pwms[2], &pwms[3], &pwms[4], &pwms[5], &pwms[6], &pwms[7]);
    
    if (result == 8)
    {
      for (int i = 0; i < 8; i++)
      {
        // bound PWM
        if(pwms[i] > MAX_PWM) pwms[i] = MAX_PWM;
        if(pwms[i] < MIN_PWM) pwms[i] = MIN_PWM;

        // write to thrusters
        thrusters[i]->writeMicroseconds(pwms[i]);
      }
    }
  }

  Serial.clear();
  delay(DELAY_PERIOD);

}
