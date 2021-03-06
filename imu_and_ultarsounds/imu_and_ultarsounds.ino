#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

long duration;float d;int i=0;
int T[]={23,25,27,29,31,33,35,37,39,41};
int E[]={22,24,26,28,30,32,34,36,38,40};

ros::NodeHandle  nh;

sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
ros::Publisher pub_range1("/ultrasound1", &range_msg1);
ros::Publisher pub_range2("/ultrasound2", &range_msg2);
ros::Publisher pub_range3("/ultrasound3", &range_msg3);

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation",&orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro",&angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl",&linear_acceleration);

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

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

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();        
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  nh.initNode();
  nh.advertise(imu_ori);
  nh.advertise(imu_gyr);
  nh.advertise(imu_acc);
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
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();    
  if (devStatus == 0) {    
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

long range_time;

void loop()
{
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
            
            // display quaternion values in easy matrix form: w x y z
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
    }

  //nh.spinOnce();
}
