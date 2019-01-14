#include <Arduino.h>
#define USE_USBCON // This is required for rosserial to work with Arduino DUE, comment for the other boards

// Have in mind, that after defining USE_USBCON, 3 seconds delay is introduced by ArduinoHardware.h in order to allow for scetch upload: 
// #if defined(USE_USBCON)
//       // Startup delay as a fail-safe to upload a new sketch
//       delay(3000); 
// #endif
// So, when you start the rosserial communication on Arduino DUE, You will get the following error at least once:

// [ERROR] [1498484524.793356]: Unable to sync with device; possible link problem or link 
// software version mismatch such as hydro rosserial_python with groovy Arduino

// Actually, despite of what the error message says, in this case it means just "Timeout":
// rosserial/SerialClient.py (found at https://github.com/ros-drivers/rosser... ) line 432, 
// it appears that the given Error is simply indicative of sync timing out. 
// After the 3 seconds delay passes, it should connect and work as expected.

#include "ros.h"
#include "ros/time.h"
#include "Ardu_Due_Leg_Ctrl_Config.h"
//header file for imu
#include "archie_msgs/Imu.h"
#include "Imu.h"

#define IMU_PUBLISH_RATE 20 //hz

#define  DEBUG_RATE 5

ros::NodeHandle nh;

archie_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
void publishIMU();
void setup() {
  // put your setup code here, to run once:

    nh.initNode();
    // nh.getHardware()->setBaud(115200);
    nh.getHardware()->setBaud(57600); 
    // nh.subscribe(pid_sub);
    // nh.subscribe(cmd_sub);
    // nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("ARCHIE LEG CONTROLLER CONNECTED");
    delay(1);
}

void loop() {
  // put your main code here, to run repeatedly:

    static unsigned long prev_imu_time = 0;
    //static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

      //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

      nh.spinOnce(); //TODO Make it spin with the rate required to keep the connection alive, not so fast
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}