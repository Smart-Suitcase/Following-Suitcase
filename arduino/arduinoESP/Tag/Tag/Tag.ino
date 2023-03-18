#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <ros.h>
#include <std_msgs/Float64.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

ros::NodeHandle nh;
std_msgs::Float64 range_msg;
ros::Publisher range_pub("range", &range_msg);

void newRange();
void newDevice(DW1000Device*);
void inactiveDevice(DW1000Device*);

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

void setup() {
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(range_pub);
  
  // Init the DW1000 module
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  
  // Start ranging as a tag
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  // Attach callback functions
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
}

void loop() {
  // Publish the range if available
  DW1000Ranging.loop();
  if (range_msg.data != 0.0) {
    range_pub.publish(&range_msg);
    range_msg.data = 0.0;
  }
  nh.spinOnce();
}

void newRange() {
  // Get the range and convert it to meters
  double range = DW1000Ranging.getDistantDevice()->getRange()+0.5;
  range_msg.data = range;
}

void newDevice(DW1000Device *device) {
  // Do nothing
}

void inactiveDevice(DW1000Device *device) {
  // Do nothing
}
