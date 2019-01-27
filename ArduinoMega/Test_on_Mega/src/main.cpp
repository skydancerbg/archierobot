#include <Arduino.h>

//#define USE_USBCON // This is required for rosserial to work with Arduino DUE, comment for the other boards

#include "ros.h"
#include "ros/time.h"

// message header files
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Int32MultiArray.h"

// #define CALLBACK_MESSAGE_TYPE std_msgs::Int32MultiArray
#define CALLBACK_MESSAGE_TYPE sensor_msgs::ChannelFloat32
// #define CALLBACK_MESSAGE_TYPE trajectory_msgs::JointTrajectory



// prototype
void messageCb( const CALLBACK_MESSAGE_TYPE& toggle_msg);

// ros::NodeHandle_<ArduinoHardware, 3, 3, 2048, 2048> nh; 
////////////////!!!!!!!!!!!!!!!//////////////////////////
ros::NodeHandle_<ArduinoHardware, 1, 1, 2048, 128> nh; 
// ros::NodeHandle nh;

ros::Subscriber<CALLBACK_MESSAGE_TYPE> sub("control/jointTrajectory", &messageCb );

 
void setup() {

  pinMode(13, OUTPUT);

    nh.initNode();

////////////////////////////////////////////////////////////////////////////////////////////////
    // nh.getHardware()->setBaud(115200);
    nh.getHardware()->setBaud(57600); 
    // nh.getHardware()->setBaud(38400); 
////////////////////////////////////////////////////////////////////////////////////////////////

    nh.subscribe(sub);
 
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("ARCHIE MEGA TEST RIG CONNECTED");
    delay(1);

}

////////////////////////loop //////////////////////////////
void loop() {
    
    nh.spinOnce(); 

}
///////////END -loop- /////////////////////////////////

///////////////////////////////////////////////////////////////

void messageCb(const CALLBACK_MESSAGE_TYPE& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the LED
  nh.loginfo("blink-----------------");
}



