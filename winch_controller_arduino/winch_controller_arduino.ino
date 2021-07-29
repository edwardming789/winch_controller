#include <ros.h>
#include <winch_controller/WinchTrigger.h>

#define ON 0
#define OFF 1

int up = 2;
int down = 4;

ros::NodeHandle  nh;
using winch_controller::WinchTrigger;

void winch_up_callback(const WinchTrigger::Request & req, WinchTrigger::Response & res){
  digitalWrite(13, HIGH-digitalRead(13));
  winch_set_status(ON,OFF);
  nh.loginfo("winch up"); 
  res.result = true; 
}

void winch_down_callback(const WinchTrigger::Request & req, WinchTrigger::Response & res){
  digitalWrite(13, HIGH-digitalRead(13));
  winch_set_status(OFF,ON);
  nh.loginfo("winch down");
  res.result = true;   
}

void winch_stop_callback(const WinchTrigger::Request & req, WinchTrigger::Response & res){
  digitalWrite(13, HIGH-digitalRead(13));
  winch_set_status(OFF,OFF);
  nh.loginfo("winch stop");
  res.result = true;   
}

void winch_set_status(unsigned char status_1, unsigned char status_2){
  digitalWrite(up, status_1);
  digitalWrite(down, status_2);
}

ros::ServiceServer<WinchTrigger::Request, WinchTrigger::Response> server1("winch/winch_up", &winch_up_callback);
ros::ServiceServer<WinchTrigger::Request, WinchTrigger::Response> server2("winch/winch_down", &winch_down_callback);
ros::ServiceServer<WinchTrigger::Request, WinchTrigger::Response> server3("winch/winch_stop", &winch_stop_callback);

void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
  pinMode(up,OUTPUT);
  pinMode(down,OUTPUT);

  winch_set_status(OFF,OFF);
  
  nh.initNode();
  nh.advertiseService(server1);
  nh.advertiseService(server2);
  nh.advertiseService(server3);

}

void loop() {
  nh.spinOnce();
  delay(1);
}
