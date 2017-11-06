#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

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
/*ros::Publisher pub_range4("/ultrasound4", &range_msg);
ros::Publisher pub_range5("/ultrasound5", &range_msg);
ros::Publisher pub_range6("/ultrasound6", &range_msg);
ros::Publisher pub_range7("/ultrasound7", &range_msg);
ros::Publisher pub_range8("/ultrasound8", &range_msg);
ros::Publisher pub_range9("/ultrasound9", &range_msg);
ros::Publisher pub_range10("/ultrasound10", &range_msg);*/

//char frameid[] = "/ultrasound";

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
}

long range_time;

void loop()
{
  if ( millis() >= range_time ){
    
    range_msg1.range = getRange(T[0],E[0]);
    range_msg1.header.stamp = nh.now();
    pub_range1.publish(&range_msg1);

    range_msg2.range = getRange(T[1],E[1]);
    range_msg2.header.stamp = nh.now();
    pub_range2.publish(&range_msg2);

    range_msg3.range = getRange(T[2],E[2]);
    range_msg3.header.stamp = nh.now();
    pub_range3.publish(&range_msg3);
        
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

  nh.spinOnce();
}

