#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

int rwida   = 7;
int rwida2 = 9;
int rotation = 8;
boolean th=false;
ros::NodeHandle nh;

void cmdVelCB( const geometry_msgs::Twist& twist)
{    
  //Serial.println("ana f cmd velo");

 //Serial.println(twist.linear.z);
  /*if(twist.angular.z > 0)
  {
      
  }
  else if(twist.angular.z < 0)
  {
      
  } */
  if(twist.linear.x > 0)
  {
    analogWrite(rwida,0);
    analogWrkite(rwida2,0);
    delay(5500);
    digitalWrite(rotation,HIGH);
    delay(100);
    analogWrite(rwida,50);
    analogWrite(rwida2,50);
  
  }
  else if(twist.linear.x < 0)
  {
    analogWrite(rwida,0);
    analogWrite(rwida2,0);
    delay(5500);
    digitalWrite(rotation,LOW);
    delay(100);
    analogWrite(rwida,50);
    analogWrite(rwida2,50);



  }
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

void setup(){
  //Serial.begin(57600);
  pinMode(rwida, OUTPUT);
  pinMode(rwida2, OUTPUT);

  pinMode(rotation, OUTPUT);
      digitalWrite(rotation,HIGH);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  nh.initNode();
  nh.subscribe(subCmdVel);
}

void loop(){
  nh.spinOnce();
}
