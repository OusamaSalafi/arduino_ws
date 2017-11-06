#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <TimerOne.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define L_h1 3
#define L_h2 19
#define R_h1 18
#define R_h2 17
#define LF_pwm 4
#define LR_pwm 5
#define RF_pwm 6
#define RR_pwm 7
#define LF_IN 42
#define LR_IN 43
#define RF_IN 44
#define RR_IN 45

#define LOOP_TIME 1000000

MPU6050 mpu;

long duration;
int i=0;
int T[]={23,25,27,29,31,33,35,37,39,41};
int E[]={22,24,26,28,30,32,34,36,38,40};
int nl = 0;
int nr = 0;
unsigned int counter_left = 0;
unsigned int counter_right = 0;
float d;
float left_wheel_vel = 0;
float right_wheel_vel = 0;
float L = 0.45;
float diameter = 0.254;
float radius = diameter / 2;
float pi = 3.14159265;

ros::NodeHandle  nh;

sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
ros::Publisher pub_range1("/ultrasound1", &range_msg1);
ros::Publisher pub_range2("/ultrasound2", &range_msg2);
ros::Publisher pub_range3("/ultrasound3", &range_msg3);
/*ros::Publisher pub_range4("/ultrasound4", &range_msg);
ros::Publisher pub_range5("/ultrasound5", &range_msg);
ros::Publisher pub_range6("/ultrasound6", &range_msg);
ros::Publisher pub_range7("/ultrasound7", &range_msg);
ros::Publisher pub_range8("/ultrasound8", &range_msg);
ros::Publisher pub_range9("/ultrasound9", &range_msg);
ros::Publisher pub_range10("/ultrasound10", &range_msg);*/

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation",&orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro",&angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl",&linear_acceleration);

char frameid[] = "/base_link";
char child[] = "/imu_frame";

float getRange(int T,int E)
{
  digitalWrite(T,HIGH);
  delayMicroseconds(10);
  digitalWrite(T,LOW);
  duration=pulseIn(E,HIGH);
  d=duration/58;
  delayMicroseconds(5);
  return d/100;
}

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  if(twist.linear.x > 0)
  {
 
  }
  else if(twist.linear.x < 0)
  {

  }
  if(twist.angular.z > 0)
  {

  }
  else if(twist.angular.z < 0)
  {

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
  nh.spinOnce();
  delay(100);
  counter_left = 0;
  counter_right = 0;
  Timer1.attachInterrupt(timerIsr);
}

void setup() {
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  /*nh.advertise(pub_range4);
  nh.advertise(pub_range5);
  nh.advertise(pub_range6);
  nh.advertise(pub_range7);
  nh.advertise(pub_range8);
  nh.advertise(pub_range9);
  nh.advertise(pub_range10);*/

  range_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg1.header.frame_id =  "/ultrasound1";
  range_msg1.field_of_view = 1.570796;
  range_msg1.min_range = 0.02;
  range_msg1.max_range = 4.0;
  
  range_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg2.header.frame_id =  "/ultrasound2";
  range_msg2.field_of_view = 1.570796;
  range_msg2.min_range = 0.02;
  range_msg2.max_range = 4.0;
  
  range_msg3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg3.header.frame_id =  "/ultrasound3";
  range_msg3.field_of_view = 1.570796;
  range_msg3.min_range = 0.02;
  range_msg3.max_range = 4.0;

  for(i=0;i<3;i++){
    pinMode(T[i], OUTPUT);
    digitalWrite(T[i], LOW);
    pinMode(E[i], INPUT);
  }

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  nh.initNode();
  //broadcaster.init(nh);
  nh.advertise(imu_ori);
  nh.advertise(imu_gyr);
  nh.advertise(imu_acc);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }

  pinMode(LF_pwm, OUTPUT);
  pinMode(LR_pwm, OUTPUT);
  pinMode(RF_pwm, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(LF_IN, OUTPUT);
  pinMode(LR_IN, OUTPUT);
  pinMode(RF_IN, OUTPUT);
  pinMode(RR_IN, OUTPUT);

  analogWrite(LF_pwm, 0);
  analogWrite(LR_pwm, 0);
  analogWrite(RF_pwm, 0);
  analogWrite(RR_pwm, 0);

  digitalWrite(LF_IN, HIGH);
  digitalWrite(LR_IN, HIGH);
  digitalWrite(RF_IN, HIGH);
  digitalWrite(RR_IN, HIGH);
      
  pinMode(L_h1, INPUT);
  pinMode(L_h2, INPUT);
  pinMode(R_h1, INPUT);
  pinMode(R_h2, INPUT);

  Timer1.initialize(LOOP_TIME);

  attachInterrupt(1, leftCount, CHANGE);
  attachInterrupt(4, leftCount, CHANGE);
  attachInterrupt(5, rightCount, CHANGE);

  //nh.subscribe(subCmdVel);
  nh.advertise(wheel_vel_pub);

  Timer1.attachInterrupt(timerIsr);
}

unsigned long range_time;

void loop()
{
  nh.spinOnce();
  if ( millis() >= range_time ){

    range_msg1.range = getRange(T[0],E[0]);
    range_msg1.header.stamp = nh.now();
    pub_range1.publish(&range_msg1);
    nh.spinOnce();

    range_msg2.range = getRange(T[1],E[1]);
    range_msg2.header.stamp = nh.now();
    pub_range2.publish(&range_msg2);
    nh.spinOnce();

    range_msg3.range = getRange(T[2],E[2]);
    range_msg3.header.stamp = nh.now();
    pub_range3.publish(&range_msg3);
    nh.spinOnce();

    /*range_msg.range = getRange(T[3],E[3]);
    range_msg.header.stamp = nh.now();
    pub_range4.publish(&range_msg);

    range_msg.range = getRange(T[4],E[4]);
    range_msg.header.stamp = nh.now();
    pub_range5.publish(&range_msg);

    range_msg.range = getRange(T[5],E[5]);
    range_msg.header.stamp = nh.now();
    pub_range6.publish(&range_msg);

    range_msg.range = getRange(T[6],E[6]);
    range_msg.header.stamp = nh.now();
    pub_range7.publish(&range_msg);

    range_msg.range = getRange(T[7],E[7]);
    range_msg.header.stamp = nh.now();
    pub_range8.publish(&range_msg);

    range_msg.range = getRange(T[8],E[8]);
    range_msg.header.stamp = nh.now();
    pub_range9.publish(&range_msg);

    range_msg.range = getRange(T[9],E[9]);
    range_msg.header.stamp = nh.now();
    pub_range10.publish(&range_msg);*/
    
    range_time =  millis() + 50;
  }
  
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x01)
  {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
        mpu.getFIFOBytes(fifoBuffer, packetSize);
  
        fifoCount -= packetSize;
  
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
            orientation.x = q.x;
            orientation.y = q.y;
            orientation.z = q.z;
            orientation.w = q.w;
            imu_ori.publish(&orientation);
            nh.spinOnce();
            delay(100);
  
            angular_velocity.x = ypr[0];
            angular_velocity.y = ypr[1];
            angular_velocity.z = ypr[2];
            imu_gyr.publish(&angular_velocity);
            nh.spinOnce();
            delay(100);
  
            linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
            imu_acc.publish(&linear_acceleration);
            nh.spinOnce();
            delay(100);
  
//            t.header.frame_id = frameid;
//            t.child_frame_id = child;
//            t.transform.translation.x = 0.5;
//            t.transform.rotation.x = q.x;
//            t.transform.rotation.y = q.y;
//            t.transform.rotation.z = q.z;
//            t.transform.rotation.w = q.w;
//            t.header.stamp = nh.now();
//            broadcaster.sendTransform(t);  
    }
}
