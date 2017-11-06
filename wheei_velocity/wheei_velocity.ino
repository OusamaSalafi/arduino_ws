//////////////////LIBRARIES///////////////////
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//////////////////LIBRARIES///////////////////

//////////////////ROS Vars and OBJECTS///////////////////
MPU6050 mpu;

long duration;float d;int i=0;
int T[]={23,25,27,29,31,33,35,37,39,41}; 
int E[]={22,24,26,28,30,32,34,36,38,40}; 
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
char frameid[] = "/base_link";
char child[] = "/imu_frame";

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
//////////////////ROS Vars and OBJECTS///////////////////

void dmpDataReady() {
    mpuInterrupt = true;
}

//////////////////Defines///////////////////
#define L_h1 3
#define L_h2 19
#define R_h1 18
#define R_h2 17
//////////////////Defines///////////////////

ros::NodeHandle  nh;

sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
ros::Publisher pub_range1("/ultrasound1", &range_msg1);
ros::Publisher pub_range2("/ultrasound2", &range_msg2);
ros::Publisher pub_range3("/ultrasound3", &range_msg3);

geometry_msgs::Twist wheel_vel;
ros::Publisher wheel_vel_pub("/wheel_velocity", &wheel_vel);

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation",&orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro",&angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl",&linear_acceleration);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

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
  //delay(100);
  counter_left = 0;
  counter_right = 0;
}

void setup() {
  nh.initNode();  
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  
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
  
  pinMode(L_h1, INPUT);
  pinMode(L_h2, INPUT);
  pinMode(R_h1, INPUT);
  pinMode(R_h2, INPUT);

  attachInterrupt(1, leftCount, CHANGE);
  attachInterrupt(4, leftCount, CHANGE);
  attachInterrupt(5, rightCount, CHANGE);

  nh.advertise(wheel_vel_pub);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
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
  
//  broadcaster.init(nh);
}

long range_time;
unsigned long previousMillis = 0;
const long interval = 60000;           // interval at which to blink (milliseconds)

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    timerIsr();    
  }
  else{
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
    
          orientation.x = q.x;
          orientation.y = q.y;
          orientation.z = q.z;
          orientation.w = q.w;
          imu_ori.publish(&orientation);
          nh.spinOnce();
    
          angular_velocity.x = ypr[0];
          angular_velocity.y = ypr[1];
          angular_velocity.z = ypr[2];
          imu_gyr.publish(&angular_velocity);
          nh.spinOnce();
    
          linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
          linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
          linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
          imu_acc.publish(&linear_acceleration);
          nh.spinOnce();
          
//          t.header.frame_id = frameid;
//          t.child_frame_id = child;
//          t.transform.translation.x = 0.5;
//          t.transform.rotation.x = q.x;
//          t.transform.rotation.y = q.y;
//          t.transform.rotation.z = q.z;
//          t.transform.rotation.w = q.w;
//          t.header.stamp = nh.now();
//          broadcaster.sendTransform(t);           
      }   
  }
  nh.spinOnce();
}
