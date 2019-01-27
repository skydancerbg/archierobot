#include <Arduino.h>
/////!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////
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
// make the nessesary changes if any in:  /home/..YOUR_USER_NAME../archie_ws/src/archierobot/arduino_due/firmware/ardu_due_leg_firmware/lib/config/firmware_config.h
#include "firmware_config.h"
//header files for the INU message and the IMU
#include "archie_msgs/Imu.h" //// location:  /home/..YOUR_USER_NAME../archie_ws/src/archierobot/arduino_due/firmware/ardu_due_leg_firmware/lib/ros_lib/archie_msgs/
#include "Imu.h"  //// location:  /home/..YOUR_USER_NAME../archie_ws/src/archierobot/arduino_due/firmware/ardu_due_leg_firmware/lib/imu/
// message header files
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

//#define USE_LED_BUILTIN // Uncomment to use the built in LED for debbuging!!! Comment for removing the LED for debbuging code at compile time

#define USE_IMU // Comment if no IMU is attached !!!!!!! Uncomment  if IMU is attached
#define IMU_TOPIC_NAME "raw_imu" // You can change the IMU publishing topic name here
                                          // if there are more than one IMU's on the robot
                                          // for example: "raw_imu/left_leg"

#define IMU_PUBLISH_RATE 20 //hz - times per second

#define FEEDBACKJOINT_PUBLISH_RATE 5 //hz  - times per second

// In platformio we use .cpp (not .ino) files in order for the intelisense and othe goodies to work propely.
// C requires that functions be forward declared for the compiler to compile and link them. 
// Look here for detailed explanation and how to on forward declarations:
// https://community.platformio.org/t/order-of-function-declaration/4546 

//callback forward declaration
void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& jt);

//publishIMU forward declaration
#if defined (USE_IMU) 
void publishIMU();
#endif

//publishFeedbackJoint forward declaration
void publishFeedbackJoint();

// other forward declarations
int setValuesToPoint(trajectory_msgs::JointTrajectory* trajectoire, int pointNumber, int jointNumber, float val);
void PrintFloats(int jointsCount, float *values);

///////////////////////// CHANGE PUB/SUB BUFFER SIZES HERE, BASED ON THE EXPECTED MESSAGE SIZE /////////////////////////
/////////////////////////////// http://wiki.ros.org/rosserial/Overview/Initialization /////////////////////////////////
///////////////////////////////////////////////  THE DEFAULT IS:  /////////////////////////////////////////////////////  
// ros::NodeHandle_<HardwareType, MAX_PUBLISHERS=25, MAX_SUBSCRIBERS=25, IN_BUFFER_SIZE=512, OUT_BUFFER_SIZE=512> nh;//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AFTER A LOT OF TESTING I'VE FOUND OUT THAT SOMEHOW THE PLACES OF MAX_PUBLISHERS AND MAX_SUBSCRIBERS AT THE ABOVE
// ARE SWAPPED FOR ARDUINO DUE ?????
// FIRST POSITION SETS THE NUMBER OF SUBSCRIBERS INSTEAD OF PUBLISHERS AND THE SECOND POSITION SETS THE NUMBER OF 
// PUBLISHERS INSTEAD OF SUBSCRIBERS ?????
//
// SO, YOU HAVE TO FIRST SET THE MAX_SUBSCRIBERS NUMBER IN THE FIRST POSITION AND THE MAX_PUBLISHERS NUMBER AT THE SECOND POSITIONS TO MAKE IT WORK!!!!!
// ON ARDUINO DUE IT LOOKS LIKE:
//   ros::NodeHandle_<HardwareType, MAX_SUBSCRIBERS, MAX_PUBLISHERS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> nh;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ros::NodeHandle_<ArduinoHardware, 1, 2, 1024, 1024> nh;
// ros::NodeHandle_<ArduinoHardware, 1, 2, 2048, 2048> nh;
//ros::NodeHandle_<ArduinoHardware, 1, 2, 4096, 4096> nh;
ros::NodeHandle_<ArduinoHardware,   1, 2, 8192, 8192> nh; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////// YOU CAN CHANGE THE SERIAL (RING) BUFFER SIZE IN platformio.ini ///////////////////////
///////////  LIKE THIS:
// [env:due]
// platform = atmelsam
// board = due
// framework = arduino
// build_flags = -D SERIAL_RX_BUFFER_SIZE=4096
// build_flags = -D SERIAL_TX_BUFFER_SIZE=4096
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////// ROS TOPIC PUBLISHERS ///////////////////////////////////

trajectory_msgs::JointTrajectory traj;
ros::Publisher feedbackJoint_pub("control/feedbackJoint", &traj);

///// THE IMU PUBLISHER COMPILES CONDITIONALY, IF USE_IMU IS DEFINED ABOVE
///// IT USES THE TOPIC NAME DEFINED ABOVE IN IMU_TOPIC_NAME
#if defined (USE_IMU) 
archie_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub(IMU_TOPIC_NAME, &raw_imu_msg);
#endif
///////////////////////////////////////////////////////////////////////


////////////// ROS TOPIC SUBSCRIBERS ///////////////////////////////////
ros::Subscriber<trajectory_msgs::JointTrajectory> jointTraj_sub("control/jointTrajectory", &jointTrajectoryCallback);


long int pubNumberCounter = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////CHANGE NUMBER OF PUBLISHED DUMMY JOINTS AND POINTS HERE:
const int numberOfJoints = 1;
const int maxPoints = 1;
/////////////////////////////////////////////////////////////////////

// char *jNames[12] = {(char*)"HeapYawLeft", (char*)"HeapRollLeft", (char*)"HeapPitchLeft", (char*)"KneePitchLeft", (char*)"AnklePitchLeft", (char*)"AncleRollLeft", (char*)"HeapYawRight", (char*)"HeapRollRight", (char*)"HeapPitchRight", (char*)"KneePitchRight", (char*)"AnklePitchRight", (char*)"AncleRollLeft"};
char *jNames[12] = {(char*)"j0", (char*)"j1", (char*)"j2", (char*)"j3", (char*)"j4", (char*)"j5", (char*)"j6", (char*)"j7", (char*)"j8", (char*)"j9", (char*)"j10", (char*)"j11"};
////////////////////////////////////

trajectory_msgs::JointTrajectoryPoint pointsArray[maxPoints];


void setup() {
    
    #if defined (USE_LED_BUILTIN) 
        pinMode(LED_BUILTIN, OUTPUT);
    #endif

    //INIT THE ROS NODE
    nh.initNode();

    // ESTABLISH HE SERIAL CONNECTION
    // nh.getHardware()->setBaud(115200); // While testing, Arduino Due was not able to connect at this speed!
    nh.getHardware()->setBaud(57000); // Best serial performance at this speed
    // In the ROS terminal use:
    // rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57000
    // to start the communication with the Arduino Due

    

    // SUBSCRIBE THE SUBSCRIBER
    nh.subscribe(jointTraj_sub);

    // ADVERTISE THE PUBLISHERS
    #if defined (USE_IMU) 
    nh.advertise(raw_imu_pub);
    #endif

    nh.advertise(feedbackJoint_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    // PRINT " IS CONNECTED" MESSAGE TO THE TERMINAL ON THE ROS SIDE
    // IN YOUR CODE YOU CAN USE nh.loginfo("YOUR_TEXT"); 
    // FOR PRINTING DIAGNOSTICS AND OTHER MESSAGES TO THE TERMINAL
    nh.loginfo("ARCHIE LEG CONTROLLER CONNECTED");
    
    // IMPORTANT: HERE, IN THE SETUP, IS THE ONLY PLACE IN YOUR CODE YOU ARE ALLOWED TO USE DELAY!!!
    // WE HAVE TO CALL nh.spinOnce() IN OUR LOOP, AS OFTEN, AS POSSIBLE FOR ROSSERIAL TO PROCESS THE INCOMMING MESSAGES!
    // DELAY IS A BLOCKING FUNCTION AND WILL PREVENT nh.spinOnce() FROM BEEING CALLED IN THE DESIRED RATE!
    // USE THE "BLINK WITHOUT DELAY" EXAMPLE CONCEPT OR TIMERS INSTEAD !!!!
    // WITH THIS DELAY WE GIVE SOME TIME FOR MAKING THE ROSSERIAL CONNECTION    
    delay(1);

    ///////////////////////////////////////////////////

    for (int pointIndex = 0; pointIndex < maxPoints; pointIndex++)
    {
        pointsArray[pointIndex].positions_length = numberOfJoints;
        pointsArray[pointIndex].positions = new float[numberOfJoints];
        pointsArray[pointIndex].velocities_length = numberOfJoints;
        pointsArray[pointIndex].velocities = new float[numberOfJoints];
        pointsArray[pointIndex].accelerations_length = numberOfJoints;
        pointsArray[pointIndex].accelerations = new float[numberOfJoints];
        pointsArray[pointIndex].effort_length = numberOfJoints;
        pointsArray[pointIndex].effort = new float[numberOfJoints];
    }
}
///////////END SETUP /////////////////////////////////

void loop() {
    //  VARIABLES TO HOLD THE LAST TIME WE PUBLISHED A MESSAGE OF A GIVEN TIME (IN MILISECONDS)
    //  IN ORDER TO USE IT IN THE IF BELOW IN ORDER TO SET THE RATE OF PUBLISHING FOR EACH TYPE OF MESSAGE
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_feedbackJoint_time = 0;
    
    static bool imu_is_initialized;



/////////// IMU PUBLISHING////////////////////////////////////////////////////////////

///// THE IMU PUBLISHER COMPILES CONDITIONALLY, IF USE_IMU IS DEFINED ABOVE
///// IT USES THE TOPIC NAME DEFINED ABOVE IN IMU_TOPIC_NAME
    #if defined (USE_IMU) 

    nh.spinOnce(); // PROCESS INCOMMING MESSAGES 

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

#endif

   

/////////// feedbackJoint PUBLISHING /////////////////////////////////////////////////

      //this block publishes the feedbackJoint data based on defined publish rate
    if ((millis() - prev_feedbackJoint_time) >= (1000 / FEEDBACKJOINT_PUBLISH_RATE))
    {
  
        publishFeedbackJoint();
		
		pubNumberCounter++;
        prev_feedbackJoint_time = millis();
    }

    nh.spinOnce(); // PROCESS INCOMMING MESSAGES

}
///////////END loop /////////////////////////////////


///////// ROS PUBLISHERS: ///////////////////////////////////////////////////////////////////////

///////////IMU PUBLISHER///////////////////////////////////////////

///// THE IMU PUBLISHER COMPILES CONDITIONALLY, IF USE_IMU IS DEFINED ABOVE
///// IT USES THE TOPIC NAME DEFINED ABOVE IN IMU_TOPIC_NAME
#if defined (USE_IMU) 
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
#endif


///////////FEEDBACKJOINT PUBLISHER///////////////////////////////////////////

void publishFeedbackJoint()
{
    // THE ROS JOINT TRAJECTORY MESSAGE DESCRIPTION HERE: http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html
    // IT HOLDS AN ARRAY OF POINTS, DESCRIPTION HERE: http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html

    // BECAUSE OF TE ARDUINO SPECIFICS DESCRIBED HERE: http://wiki.ros.org/rosserial/Overview/Limitations
    // THE ROSSERIAL ARDUINO SIDE HAS FEW MORE FIELDS IN THE MESSAGE
    // FIND THE ARDUINO SIDE MESSAGE DESCRIPTIONS HERE:
    // /home/..YOUR_USER_NAME../archie_ws/src/archierobot/arduino_due/firmware/ardu_due_leg_firmware/lib/ros_lib/trajectory_msgs/JointTrajectory.h
    // AND THE POINTS MESSAGE (THERE IS AN ARRAY OF POINTS INSIDE THE JointTrajectory MESSAGE) HERE: 
    // /home/..YOUR_USER_NAME../archie_ws/src/archierobot/arduino_due/firmware/ardu_due_leg_firmware/lib/ros_lib/trajectory_msgs/JointTrajectoryPoint.h

    trajectory_msgs::JointTrajectoryPoint point;

    traj.header.frame_id = "base_link";
    traj.joint_names_length=numberOfJoints;
    traj.points_length = maxPoints;
    traj.joint_names = jNames;

    traj.points = pointsArray;

	for(int pointNumber = 0; pointNumber < maxPoints; pointNumber++) 
    {
        traj.points[pointNumber].positions_length=numberOfJoints;
        traj.points[pointNumber].accelerations_length=numberOfJoints;
        traj.points[pointNumber].velocities_length=numberOfJoints;
        traj.points[pointNumber].effort_length=numberOfJoints;
        // INFO ABOUT DURIATION HERE: http://wiki.ros.org/rosserial/Overview/Time
        ros::Duration durSeconds (5,9999);
        traj.points[pointNumber].time_from_start=durSeconds;
	}

	// set dummy values to the pointS array
	for(int pointNumber = 0; pointNumber < maxPoints; pointNumber++) {
        for(int jointNumber = 0; jointNumber < numberOfJoints; jointNumber++) {
                setValuesToPoint(&traj,pointNumber,jointNumber,float(pubNumberCounter));
        }
	}

    // FINALLY, JUST BEFORE PUBLISHING, GET THE CURRENT ROS TIME WITH THIS FUNCTION
    // AND FILL THE TIME STAMP IN THE HEADER
       traj.header.stamp = nh.now();


    // PUBLISH THE MESSAGE!!!!!!!!
		feedbackJoint_pub.publish(&traj);

}


// fills a point with dummy values
int setValuesToPoint(trajectory_msgs::JointTrajectory* trajectoire, int pointNumber, int jointNumber, float val){
    trajectoire->points[pointNumber].positions[jointNumber] = val;
    trajectoire->points[pointNumber].accelerations[jointNumber] = val;
	trajectoire->points[pointNumber].velocities[jointNumber] = val;
	trajectoire->points[pointNumber].effort[jointNumber] = val;
    return 0;
}

///////// END FEEDBACKJOINT PUBLISHER RELATED//////////////////////////



///////// ROS SUBSCRIBERS:     ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////// JOINT TRAJECTORY SUBSCRIBER        //////////////////////////

void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& jt)
{
    #if defined (USE_LED_BUILTIN) 
    digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the LED
    #endif

    // nh.loginfo("In callback!!!");

        char buffer[128];

    nh.loginfo("header:");
    sprintf(buffer, "\tseq: %lu", jt.header.seq);
    nh.loginfo(buffer);
    nh.loginfo("\tstamp:");
    sprintf(buffer,"\t\tsecs: %f", jt.header.stamp.toSec());
    nh.loginfo(buffer);
    // nh.loginfo("\t\tnsecs: %lu", jt.header.stamp.toNSec());
    sprintf(buffer,"\t\tframeId: %s", jt.header.frame_id);
    nh.loginfo(buffer);

    for(unsigned long i = 0; i < jt.joint_names_length; i++)
    { 
        sprintf(buffer, "\t\t\t%s", jt.joint_names[i]);
        nh.loginfo(jt.joint_names[i]);

    } 


    nh.loginfo("points:");
    for(unsigned long pointIndex = 0; pointIndex < jt.points_length; pointIndex++)
    {

        nh.loginfo("positions:");
        PrintFloats(jt.joint_names_length, jt.points[pointIndex].positions);

        nh.loginfo("velocities:");
        PrintFloats(jt.joint_names_length, jt.points[pointIndex].velocities);

        nh.loginfo("accelerations:");
        PrintFloats(jt.joint_names_length, jt.points[pointIndex].accelerations);
    
        nh.loginfo("effort:");
        PrintFloats(jt.joint_names_length, jt.points[pointIndex].effort);

        nh.loginfo("\ttime from start:");
        sprintf(buffer, "\t\tsecs: %f", jt.points[pointIndex].time_from_start.toSec());
        nh.loginfo(buffer);
        //Only the human radable representation in seconds exists in ros_serial Arduino    
        // nh.loginfo("\t\tnsecs: %lu", jt.points[pointIndex].time_from_start.toNSec()); does not exist
        ///// ATTENTION in you need the nanosecond representation you should calculate it yourself:
                // 1 nsec. = 0.000000001 sec
        ////// OR YOU CAN OUTPUT 0 INSTEAD
        //nh.loginfo("\t\tnsecs: 0");
      
        
 }

}

 void PrintFloats(int jointsCount, float *values)
 {
    char buffer[128];
	for(int jointIndex = 0; jointIndex < jointsCount; jointIndex++)
    {
        sprintf(buffer, "\t\t\t\t%f", values[jointIndex]);
		nh.loginfo(buffer);
    }
 } 
 
///////// END JOINT TRAJECTORY SUBSCRIBER RELATED //////////////////////////


