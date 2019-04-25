#include <ArduinoHardware.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <vugc1_control/steering_values.h>
#include <std_msgs/String.h>

// 12 bit DAC
Adafruit_MCP4725 T1;
Adafruit_MCP4725 T2;

int TRQ_CENTER = 2048; //  2.5V
int TRQ_LOWER = 0;   //  0V
int TRQ_HIGHER = 4095;  // 12 bits 5V

vugc1_control::steering_values prev;
void messageSteer(const vugc1_control::steering_values &trq);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc1_control::steering_values> vugc1_control_torque_parameters("vugc1_control_torque_parameters", &messageSteer);


void messageSteer(const vugc1_control::steering_values &trq) {  
  vugc1_control::steering_values safe;
  safe.trq_1 = min(TRQ_HIGHER, max(TRQ_LOWER, trq.trq_1));
  safe.trq_2 = min(TRQ_HIGHER, max(TRQ_LOWER, trq.trq_2));

  T1.setVoltage(safe.trq_1, false);
  T2.setVoltage(safe.trq_2, false);
  delay(50);

  // reset to center
  T1.setVoltage(TRQ_CENTER, false);
  T2.setVoltage(TRQ_CENTER, false);

  prev.trq_1 = safe.trq_1;
  prev.trq_2 = safe.trq_2;
}

void setup() {
  Serial.begin(9600);
  T1.begin(0x62);
  T2.begin(0x63);

  T1.setVoltage(TRQ_CENTER, false);
  T2.setVoltage(TRQ_CENTER, false);

  nh.initNode();
  nh.subscribe(vugc1_control_torque_parameters);

  prev.trq_1 = TRQ_CENTER;
  prev.trq_2 = TRQ_CENTER;

}

void loop() {
  nh.spinOnce();
}
