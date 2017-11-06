#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <TimerOne.h>

#define L_h1 2
#define L_h2 3
#define R_h1 21
#define R_h2 20
#define LF_pwm 4
#define LR_pwm 5
#define RF_pwm 6
#define RR_pwm 7
#define LF_IN1 42
#define LF_IN2 43
#define LR_IN1 44
#define LR_IN2 45
#define RF_IN1 46
#define RF_IN2 47
#define RR_IN1 48
#define RR_IN2 49

#define LOOP_TIME 1000000

int nl = 0;
int nr = 0;
unsigned int counter_left = 0;
unsigned int counter_right = 0;
float left_wheel_vel = 0;
float right_wheel_vel = 0;
float L = 0.45;
float diameter = 0.254;
float radius = diameter / 2;
float pi = 3.14159265;

unsigned char last_left_val_a = 0;
unsigned char last_left_val_b = 0;
unsigned char last_right_val_a = 0;
unsigned char last_right_val_b = 0;

ros::NodeHandle  nh;

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  if(twist.linear.x > 0)
  {
    analogWrite(LF_pwm,255);
    analogWrite(LR_pwm,255);
    analogWrite(RF_pwm,255);
    analogWrite(RR_pwm,255);

    digitalWrite(LF_IN1, HIGH);
    digitalWrite(LF_IN2, HIGH);
    digitalWrite(LR_IN1, HIGH);
    digitalWrite(LR_IN2, HIGH);
    digitalWrite(RF_IN1, HIGH);
    digitalWrite(RF_IN2, HIGH);
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, HIGH);
  }
  else if(twist.linear.x < 0)
  {
    analogWrite(LF_pwm,255);
    analogWrite(LR_pwm,255);
    analogWrite(RF_pwm,255);
    analogWrite(RR_pwm,255);

    digitalWrite(LF_IN1, LOW);
    digitalWrite(LF_IN2, LOW);
    digitalWrite(LR_IN1, LOW);
    digitalWrite(LR_IN2, LOW);
    digitalWrite(RF_IN1, LOW);
    digitalWrite(RF_IN2, LOW);
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, LOW);
  }
  else
  {
    analogWrite(LF_pwm,0);
    analogWrite(LR_pwm,0);
    analogWrite(RF_pwm,0);
    analogWrite(RR_pwm,0);
  }

  if(twist.angular.z > 0)
  {
    analogWrite(LF_pwm,255);
    analogWrite(LR_pwm,255);
    analogWrite(RF_pwm,255);
    analogWrite(RR_pwm,255);

    digitalWrite(LF_IN1, LOW);
    digitalWrite(LF_IN2, LOW);
    digitalWrite(LR_IN1, LOW);
    digitalWrite(LR_IN2, LOW);
    digitalWrite(RF_IN1, HIGH);
    digitalWrite(RF_IN2, HIGH);
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, HIGH);
  }
  else if(twist.angular.z < 0)
  {
    analogWrite(LF_pwm,255);
    analogWrite(LR_pwm,255);
    analogWrite(RF_pwm,255);
    analogWrite(RR_pwm,255);

    digitalWrite(LF_IN1, HIGH);
    digitalWrite(LF_IN2, HIGH);
    digitalWrite(LR_IN1, HIGH);
    digitalWrite(LR_IN2, HIGH);
    digitalWrite(RF_IN1, LOW);
    digitalWrite(RF_IN2, LOW);
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, LOW);
  }
  else
  {
    analogWrite(LF_pwm,0);
    analogWrite(LR_pwm,0);
    analogWrite(RF_pwm,0);
    analogWrite(RR_pwm,0);
  }
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

geometry_msgs::Twist wheel_vel;
ros::Publisher wheel_vel_pub("/wheel_velocity", &wheel_vel);

void leftCount(){
  char val_a = digitalRead(L_h1);
  char val_b = digitalRead(L_h2);
  if(val_a != val_b){
    nl+=2;
  }
  else{
    nl++;
  }
  if(nl == 6){
    counter_left++;
    nl=0;
  }
}

void rightCount(){
  char val_a = digitalRead(R_h1);
  char val_b = digitalRead(R_h2);
  if(val_a != val_b){
    nr+=2;
  }
  else{
    nr++;
  }
  if(nr == 6){
    counter_right++;
    nr=0;
  }
}

void timerIsr()
{
  Timer1.detachInterrupt();
  left_wheel_vel = float(counter_left)*2*pi*radius;
  right_wheel_vel = float(counter_right)*2*pi*radius;
  wheel_vel.linear.x = radius*(left_wheel_vel + right_wheel_vel)/2;
  wheel_vel.linear.y = 0;
  wheel_vel.linear.z = 0;
  wheel_vel.angular.x = 0;
  wheel_vel.angular.y = 0;
  wheel_vel.angular.z = radius*(left_wheel_vel + right_wheel_vel)/L;
  wheel_vel_pub.publish(&wheel_vel);
  counter_left = 0;
  counter_right = 0;
  Timer1.attachInterrupt(timerIsr);
}

void setup() {
  pinMode(LF_pwm, OUTPUT);
  pinMode(LR_pwm, OUTPUT);
  pinMode(RF_pwm, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(LF_IN1, OUTPUT);
  pinMode(LF_IN2, OUTPUT);
  pinMode(LR_IN1, OUTPUT);
  pinMode(LR_IN2, OUTPUT);
  pinMode(RF_IN1, OUTPUT);
  pinMode(RF_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);

  analogWrite(LF_pwm, 0);
  analogWrite(LR_pwm, 0);
  analogWrite(RF_pwm, 0);
  analogWrite(RR_pwm, 0);

  digitalWrite(LF_IN1, LOW);
  digitalWrite(LF_IN2, HIGH);
  digitalWrite(LR_IN1, LOW);
  digitalWrite(LR_IN2, HIGH);
  digitalWrite(RF_IN1, LOW);
  digitalWrite(RF_IN2, HIGH);
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, HIGH);

  pinMode(L_h1, INPUT);
  pinMode(L_h2, INPUT);
  pinMode(R_h1, INPUT);
  pinMode(R_h2, INPUT);

  last_left_val_a = digitalRead(L_h1);
  last_left_val_b = digitalRead(L_h2);
  last_right_val_a = digitalRead(R_h1);
  last_right_val_b = digitalRead(R_h2);

  Timer1.initialize(LOOP_TIME);

  attachInterrupt(0, leftCount, CHANGE);
  attachInterrupt(1, leftCount, CHANGE);
  attachInterrupt(2, rightCount, CHANGE);
  attachInterrupt(3, rightCount, CHANGE);

  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.advertise(wheel_vel_pub);

  Timer1.attachInterrupt(timerIsr);
}

void loop() {
  nh.spinOnce();
}
