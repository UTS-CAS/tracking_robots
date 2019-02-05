//#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <track_robot_msgs/Trigger.h>
//#include <ArduinoHardware.h>


ros::NodeHandle nh;
track_robot_msgs::Trigger msg;
ros::Publisher pub_trigger("track/trigger", &msg);

const int button_pin = 7;
const int led_pin = 13;


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_trigger);

  pinMode(button_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);

  digitalWrite(led_pin, HIGH);
  delay(1000);
  digitalWrite(led_pin, LOW);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool reading = false;
  while(1)
  {
    reading = digitalRead(button_pin);
    if(!reading)
    {
      digitalWrite(led_pin,HIGH);
      msg.trig = true;
    }
    else
    {
      digitalWrite(led_pin,LOW);
      msg.trig = false;
    }
    //msg.trig = reading;
    msg.header.stamp = nh.now();
    pub_trigger.publish(&msg); 
    nh.spinOnce();
    delay(20);
  }
}
