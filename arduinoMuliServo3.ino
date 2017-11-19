
//standing initial
//rostopic pub servo std_msgs/UInt16MultiArray '{data: [20,100,180,20,40,130,130,170]}' --once
//laying
//rostopic pub servo std_msgs/UInt16MultiArray '{data: [80,100,100,20,90,130,130,100]}' --once


//rosrun rosserial_python serial_node.py /dev/ttyACM2


#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/UInt16MultiArray.h"

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

void servo_cb( const std_msgs::UInt16MultiArray&  cmd_msg){
  servo1.attach(2); //attach it to pin 9
  servo2.attach(3);//attach it to pin10
  servo3.attach(4); //attach it to pin 9
  servo4.attach(5);//attach it to pin10
  servo5.attach(6); //attach it to pin 9
  servo6.attach(7);//attach it to pin10
  servo7.attach(8); //attach it to pin 9
  servo8.attach(9);//attach it to pin10

  
  servo1.write(cmd_msg.data[0]); //set servo angle, should be from 0-180  
  servo2.write(cmd_msg.data[1]); 
  servo3.write(cmd_msg.data[2]); 
  servo4.write(cmd_msg.data[3]);
  servo5.write(cmd_msg.data[4]); 
  servo6.write(cmd_msg.data[5]); 
  servo7.write(cmd_msg.data[6]); 
  servo8.write(cmd_msg.data[7]);  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

delay(600);
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
  servo5.detach();
  servo6.detach();
  servo7.detach();
  servo8.detach();
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);


  servo1.attach(2); //attach it to pin 9
  servo2.attach(3);//attach it to pin10
  servo3.attach(4); //attach it to pin 9
  servo4.attach(5);//attach it to pin10
  servo5.attach(6); //attach it to pin 9
  servo6.attach(7);//attach it to pin10
  servo7.attach(8); //attach it to pin 9
  servo8.attach(9);//attach it to pin10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
