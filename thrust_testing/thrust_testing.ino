#include <ros.h>
#include <std_msgs/Float64.h>
#include <HX711.h>

HX711 scale;
ros::NodeHandle nh;
std_msgs::Float64 msg;
ros::Publisher pub ("scale", &msg);

float units;
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  scale.begin(A4, A5);
  nh.initNode();
  nh.advertise(pub);
  scale.set_scale();
  scale.tare(20);
  //scale.set_scale(397756.0);

  digitalWrite(13, HIGH);
  delay(10000);
  digitalWrite(13, LOW);
  units = scale.get_units(10);
  scale.set_scale(units/0.5);
  digitalWrite(13, HIGH);
}

boolean p = false;
void loop() {
  if (p) {
    msg.data = units/0.5;
    pub.publish(&msg);
    nh.spinOnce();
  }

//  msg.data = scale.get_units(5);
//  pub.publish(&msg);
//  nh.spinOnce();
  msg.data = scale.read();
  pub.publish(&msg);
  nh.spinOnce();
}
