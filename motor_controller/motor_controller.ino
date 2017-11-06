#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LOOP_TIME 200000

#define LR_h1 2
#define LR_h2 7
#define RR_h1 3
#define RR_h2 8
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
#define RR_IN1 49

unsigned int counter_left = 0;
unsigned int counter_right = 0;
float radius = 0.025;
float pi = 3.1415;
float L = 0.1;

ros::NodeHandle nh;

geometry_msgs::Twist odom_vel;
ros::Publisher odomVelPub("/odom_vel", &odom_vel);

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  int gain = 4000;
  float left_wheel_data = gain*(twist.linear.x - twist.angular.z*L);
  float right_wheel_data = gain*(twist.linear.x + twist.angular.z*L);
  if(left_wheel_data >= 0)
  {
    analogWrite(LR_pwm,abs(left_wheel_data));
    digitalWrite(LR_IN1, LOW);
    digitalWrite(LR_IN2, HIGH);
  }
  else
  {
    analogWrite(LR_pwm,abs(left_wheel_data));
    digitalWrite(LR_IN1, HIGH);
    digitalWrite(LR_IN2, LOW);
  }
  if(right_wheel_data >= 0)
  {
    analogWrite(RR_pwm,abs(left_wheel_data));
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, HIGH);  
  }
  else
  {
    analogWrite(RR_pwm,abs(left_wheel_data));
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, LOW);
  } 
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

void docount_left()  
{
  if (digitalRead(LR_h1) == digitalRead(LR_h2)) counter_left+=1;
  else counter_left+=2;
} 

void docount_right()
{
  if (digitalRead(RR_h1) == digitalRead(RR_h2)) counter_right+=1;
  else counter_right+=2;
} 

void timerIsr()
{
  Timer1.detachInterrupt();
  float left_wheel_vel = float(counter_left)*2*5/8;
  float right_wheel_vel = float(counter_right)*2*pi*5/8;
  odom_vel.linear.x = radius*(left_wheel_vel + right_wheel_vel)/2;
  odom_vel.linear.y = 0;
  odom_vel.linear.z = 0;
  odom_vel.angular.x = 0;
  odom_vel.angular.y = 0;
  odom_vel.angular.z = radius*(left_wheel_vel + right_wheel_vel)/L;
  odomVelPub.publish(&odom_vel);
  counter_right=0;
  counter_left=0;
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
  analogWrite(LF_pwm,0);
  analogWrite(LR_pwm, 0);
  analogWrite(RF_pwm,0);
  analogWrite(RR_pwm, 0);  
  digitalWrite(LF_IN1, LOW);
  digitalWrite(LF_IN2, HIGH);
  digitalWrite(LR_IN1, LOW);
  digitalWrite(LR_IN2, HIGH);
  digitalWrite(RF_IN1, LOW);
  digitalWrite(RF_IN2, HIGH);
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, HIGH);

  pinMode(LR_h1,INPUT_PULLUP);
  pinMode(LR_h2,INPUT);  
  pinMode(RR_h1,INPUT_PULLUP);
  pinMode(RR_h2,INPUT);
  Timer1.initialize(LOOP_TIME); 
  attachInterrupt(0,docount_left,RISING);
  attachInterrupt(1,docount_right,RISING);

  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.advertise(odomVelPub);

  Timer1.attachInterrupt(timerIsr);
}

void loop() {
  nh.spinOnce();
}
