#ifndef SILVERGOOSE_DEFINES_
#define SILVERGOOSE_DEFINES_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID_A                       1       
#define DXL_LEFT_ID_B                       2      
#define DXL_RIGHT_ID_A                      3       
#define DXL_RIGHT_ID_B                      4     
#define DXL_HEAD_ID_LOW                     5     
#define DXL_HEAD_ID_MID                     6     
#define DXL_HEAD_ID_HIGH                    7       
#define BAUDRATE                            1000000 // baurd rate of Dynamixel
#define DEVICENAME                          ""      // no need setting on OpenCR

#define CONTROL_MOTOR_SPEED_PERIOD          30   //hz
#define IMU_PUBLISH_PERIOD                  200  //hz
#define CMD_VEL_PUBLISH_PERIOD              30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD    30   //hz
#define VERSION_INFORMATION_PUBLISH_PERIOD  1    //hz 

#define WHEEL_NUM                        2
#define WHEEL_RADIUS                     0.48463         // meter
#define WHEEL_SEPARATION                 0.37434           // meter (SILVER GOOSE :  )
#define TURNING_RADIUS                   0.18717          // meter (SILVER GOOSE :  )
#define ROBOT_RADIUS                     0.45           // meter (SILVER GOOSE :  )
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define NECK_LENGTH                      0.0705000000  // 
#define NECK_INVERSE_LENGTH             14.1843971631  // 
#define HEAD_LENGTH                      0.0290000000  // 
#define HEAD_LOC_Z                       0.1614520562  //
#define HEAD_LOC_X                       0.0767500000  //
#define NECK_BASE_Z                      0.0617500000  // Axle to Neck
#define NECK_BASE_X                      0.0767500000  // Axle to Neck

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

#define VELOCITY_CONSTANT_VALUE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882
                                                        
#define LINEAR                           0
#define ANGULAR                          1

#define MAX_LINEAR_VELOCITY              0.25   // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             1.5   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define TICK2RAD                         0.0007669905  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define TICK2RPM                         0.00732422238   // 
#define TICK2SERVO                       0.063967

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14159      // 180 degree

#define VELOCITY_UNIT                    2

double ticks2Dist = TICK2RAD*WHEEL_RADIUS;
double dist2Ticks = 1/ticks2Dist;

#endif // SILVERGOOSE_DEFINES_
