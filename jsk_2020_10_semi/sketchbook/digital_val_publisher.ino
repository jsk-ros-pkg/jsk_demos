#include <math.h>
#include <ros.h>
ros::NodeHandle  nh;

#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>

#define DIN_PIN 2
std_msgs::Int16 pin_msg; 
ros::Publisher arduino_pub("arduino", &pin_msg);

void pin_setup()
{
  pin_msg.data = 0;
  nh.advertise(arduino_pub);
}

void pin_loop()
{
  pinMode( DIN_PIN, INPUT_PULLUP );
  pin_msg.data = digitalRead(DIN_PIN);
  arduino_pub.publish(&pin_msg);
}

void setup()
{
  nh.initNode();
  pin_setup();
}

unsigned long timer = 0;
void loop()
{
  unsigned long now = millis();
  // measure and publich pin value every 1000 milliseconds
  if ( (now - timer) > 50 ) {
    pin_loop();
    timer = now;
  }
  nh.spinOnce();
}
