#include <DynamixelWorkbench.h>
#include <IMU.h>
#include "silvergoose_defines.h"
#include "silvergoose_diagnosis.h"

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    

#define BAUDRATE  1000000
//#define DEBUG_IMU 1

DynamixelWorkbench dxl_wb;
cIMU    IMU;
double velLeft_current, velLeft_average, velLeft_output, velLeft_goal;
double velRight_current, velRight_average, velRight_output, velRight_goal;

uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;

int32_t current_tick[2] = {0, 0};
int32_t last_tick[WHEEL_NUM] = {0.0, 0.0};
int32_t last_diff_tick[WHEEL_NUM] = {0.0, 0.0};

unsigned long prev_update_time;
unsigned long commandTime = 0;

float odom_pose[3];
double odom_vel[3];

void setup() 
{
  Serial.begin(115200);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.

  delay(500);
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  IMU.begin();
  delay(100);
  
//  Serial.println("Setup 0");
  dxl_wb.ping(DXL_LEFT_ID_A);
  dxl_wb.ping(DXL_LEFT_ID_B);
  dxl_wb.ping(DXL_RIGHT_ID_A);
  dxl_wb.ping(DXL_RIGHT_ID_B);
  dxl_wb.ping(DXL_HEAD_ID_LOW);
  dxl_wb.ping(DXL_HEAD_ID_MID);
  dxl_wb.ping(DXL_HEAD_ID_HIGH);
//  Serial.println("Setup 1");
  dxl_wb.wheelMode(DXL_LEFT_ID_A);
  dxl_wb.wheelMode(DXL_LEFT_ID_B);
  dxl_wb.wheelMode(DXL_RIGHT_ID_A);
  dxl_wb.wheelMode(DXL_RIGHT_ID_B);
//  Serial.println("Setup 2");
  dxl_wb.jointMode(DXL_HEAD_ID_LOW);
  dxl_wb.jointMode(DXL_HEAD_ID_MID);
  dxl_wb.jointMode(DXL_HEAD_ID_HIGH);
//  Serial.println("Setup 3");
}

void loop() 
{
  imu_update();
  position_head(0.0);
  motorDriver();
  if(Serial.available()){
    char input = Serial.read();
    if(input=='1'){
      driveTest(1);
    }else if(input=='2'){
      driveTest(2);
    }else if(input=='3'){
      driveTest(3);
    }else if(input=='4'){
      driveTest(4);
    }
  }
  if(commandTime < millis()){
    velRight_goal = 0;
    velLeft_goal = 0;
  }
  battery_state = updateVoltageCheck(true);
//  checkMotorStatus();
  delay(2);
}

/*******************************************************************************
* Silvergoose test drive using push buttons
*******************************************************************************/
void readEncoder(int32_t &left_value, int32_t &right_value){
  left_value = dxl_wb.itemRead(DXL_LEFT_ID_A, "Present_Position");
  right_value = dxl_wb.itemRead(DXL_RIGHT_ID_A, "Present_Position");
//  Serial.print("left_value");
//  Serial.println(left_value);
//  Serial.print("right_value");
//  Serial.println(right_value);
}

void checkMotorStatus(){
  int val;
  val = dxl_wb.itemRead(1, "Present_Position");  Serial.print("p1  ");  Serial.println(val);
  val = dxl_wb.itemRead(2, "Present_Position");  Serial.print("p2  ");  Serial.println(val);
  val = dxl_wb.itemRead(3, "Present_Position");  Serial.print("p3  ");  Serial.println(val);
  val = dxl_wb.itemRead(4, "Present_Position");  Serial.print("p4  ");  Serial.println(val);
  val = dxl_wb.itemRead(5, "Present_Position");  Serial.print("p5  ");  Serial.println(val);
  val = dxl_wb.itemRead(5, "Present_Position");  Serial.print("p6  ");  Serial.println(val);
  val = dxl_wb.itemRead(6, "Present_Position");  Serial.print("p7  ");  Serial.println(val);

  val = dxl_wb.itemRead(1, "Hardware_Error_Status");  Serial.print("e1  ");  Serial.println(val);
  val = dxl_wb.itemRead(2, "Hardware_Error_Status");  Serial.print("e2  ");  Serial.println(val);
  val = dxl_wb.itemRead(3, "Hardware_Error_Status");  Serial.print("e3  ");  Serial.println(val);
  val = dxl_wb.itemRead(4, "Hardware_Error_Status");  Serial.print("e4  ");  Serial.println(val);
  val = dxl_wb.itemRead(5, "Hardware_Error_Status");  Serial.print("e5  ");  Serial.println(val);
  val = dxl_wb.itemRead(5, "Hardware_Error_Status");  Serial.print("e6  ");  Serial.println(val);
  val = dxl_wb.itemRead(6, "Hardware_Error_Status");  Serial.print("e7  ");  Serial.println(val);
}

void motorDriver()
{
  //velRight_current, velRight_output, velRight_goal;velLeft_current, velLeft_output, velLeft_goal;
  last_tick[LEFT] = current_tick[LEFT];
  last_tick[RIGHT] = current_tick[RIGHT];
  unsigned long update_time = micros();
  readEncoder(current_tick[LEFT], current_tick[RIGHT]);
  last_diff_tick[LEFT] = current_tick[LEFT] - last_tick[LEFT];
  last_diff_tick[RIGHT] = current_tick[RIGHT] - last_tick[RIGHT];

  float deltaTime = (update_time - prev_update_time)/1000000.0;
  prev_update_time = update_time;
  if (deltaTime <= 0){
    return;
  }

//  float estimateRADIANS_RIGHT = TICK2RAD*last_diff_tick[RIGHT]/deltaTime; //RADIANS PER SEC
//  float estimateRADIANS_LEFT = -TICK2RAD*last_diff_tick[LEFT]/deltaTime; //RADIANS PER SEC
//  float estimateRPM_RIGHT = TICK2RPM*last_diff_tick[RIGHT]/deltaTime; //RPM
//  float estimateRPM_LEFT = -TICK2RPM*last_diff_tick[LEFT]/deltaTime; //RPM
  float estimateSERVO_RIGHT = TICK2SERVO*last_diff_tick[RIGHT]/deltaTime; // in servo speed units
  float estimateSERVO_LEFT = -TICK2SERVO*last_diff_tick[LEFT]/deltaTime;  // in servo speed units
  velRight_current = 0.999*velRight_current + 0.001*estimateSERVO_RIGHT;
  velLeft_current = 0.999*velLeft_current + 0.001*estimateSERVO_LEFT;

  dxl_wb.goalSpeed(DXL_LEFT_ID_A, -velLeft_goal);//set value * 0.229 is ~RPM
  dxl_wb.goalSpeed(DXL_LEFT_ID_B, -velLeft_goal);
  dxl_wb.goalSpeed(DXL_RIGHT_ID_A, velRight_goal);
  dxl_wb.goalSpeed(DXL_RIGHT_ID_B, velRight_goal);

//  Serial.print("deltaTime (ms) ");
//  Serial.println(deltaTime*1000);
  
//  Serial.print("estimateRADIANS_RIGHT");
//  Serial.println(estimateRADIANS_RIGHT);
//  Serial.print("estimateRPM_RIGHT");
//  Serial.println(estimateRPM_RIGHT);

//  Serial.print("velLeft_current");
//  Serial.println(velLeft_current);
//  Serial.print("velRight_current");
//  Serial.println(velRight_current);
}

/*******************************************************************************
* Silvergoose test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{
//  Serial.println(buttons);
  commandTime = millis()+1000;

  if (buttons == 1)  
  {
//    Serial.println("Test Forward!");
    velRight_goal = 100;
    velLeft_goal = 100;
  }
  else if (buttons == 2)
  {
//    Serial.println("Test Left!");
    velRight_goal = 100;
    velLeft_goal = -100;
  }
  else if (buttons == 3)
  {
//    Serial.println("Test Right!");
    velRight_goal = -100;
    velLeft_goal = 100;
  }
  else if (buttons == 4)
  {
//    Serial.println("Test Backward!");
    velRight_goal = -100;
    velLeft_goal = -100;
  }
}

/*******************************************************************************
* Silvergoose adjust head to angle relative to global tf with IMU correction
*******************************************************************************/
float eye_angle, shoulder_x, shoulder_z, neck_x, neck_z, est_neckMID,
      est_neckLOW, est_neckHIGH, pitch, constrained_pitch;
int neckMID_pose, neckLOW_pose, neckHIGH_pose;
void position_head(float angle)
{
  constrained_pitch = radians(constrain(IMU.rpy[1],-25,45)); // safety limit on the neck motion
  pitch = radians(IMU.rpy[1]);
  shoulder_x = NECK_BASE_X*cos(constrained_pitch) - NECK_BASE_Z*sin(constrained_pitch);
  shoulder_z = NECK_BASE_Z*cos(constrained_pitch) + NECK_BASE_X*sin(constrained_pitch);
  neck_x = HEAD_LOC_Z - shoulder_z;
  neck_z = shoulder_x - HEAD_LOC_X;
  est_neckMID = 2*asin( NECK_INVERSE_LENGTH*(0.5*sqrt(neck_x*neck_x + neck_z*neck_z)));
  est_neckHIGH = M_PI - (atan2(neck_x,neck_z) + 0.5*(M_PI - est_neckMID));
  est_neckLOW = constrained_pitch - est_neckHIGH + est_neckMID;
  est_neckHIGH += pitch - constrained_pitch; //Keeps the head level even at the safety limit
  est_neckHIGH += angle; //add in client set head tilt
 
  neckLOW_pose = constrain(-est_neckLOW*2048.0/M_PI + 2560, 1024, 3072);
  neckMID_pose = constrain(-est_neckMID*2048.0/M_PI + 3072, 1024, 3072); 
  neckHIGH_pose = constrain(-est_neckHIGH*2048.0/M_PI + 2048, 1024, 3072);

  dxl_wb.goalPosition(DXL_HEAD_ID_LOW, neckLOW_pose);
  dxl_wb.goalPosition(DXL_HEAD_ID_MID, neckMID_pose);
  dxl_wb.goalPosition(DXL_HEAD_ID_HIGH, neckHIGH_pose);

//  dxl_wb.goalPosition(DXL_HEAD_ID_LOW, 2048);
//  dxl_wb.goalPosition(DXL_HEAD_ID_MID, 2048);
//  dxl_wb.goalPosition(DXL_HEAD_ID_HIGH, 1536);

//  Serial.print("IMU.rpy[0]: ");
//  Serial.println(IMU.rpy[0]);
//  Serial.print("IMU.rpy[1]: ");
//  Serial.println(IMU.rpy[1]);
//  Serial.print("IMU.rpy[2]: ");
//  Serial.println(IMU.rpy[2]);
//  Serial.print("shoulder_x: ");
//  Serial.println(1000*(shoulder_x));
//  Serial.print("shoulder_z: ");
//  Serial.println(1000*(shoulder_z));
//  Serial.print("neck_x: ");
//  Serial.println(1000*(neck_x));
//  Serial.print("neck_z: ");
//  Serial.println(1000*(neck_z));
  
//  Serial.print("est_neckLOW: ");
//  Serial.println(degrees(est_neckLOW));
//  Serial.print("est_neckMID: ");
//  Serial.println(degrees(est_neckMID));
//  Serial.print("est_neckHIGH: ");
//  Serial.println(degrees(est_neckHIGH));
//  
//  Serial.print("neckLOW_pose: ");
//  Serial.println((neckLOW_pose));
//  Serial.print("neckMID_pose: ");
//  Serial.println((neckMID_pose));
//  Serial.print("neckHIGH_pose: ");
//  Serial.println((neckHIGH_pose));
  
//#define NECK_LENGTH                      0.0705000000  // 
//#define NECK_INVERSE_LENGTH             14.1843971631  // 
//#define HEAD_LENGTH                      0.0290000000  // 
//#define HEAD_LOC_Z                       0.1614520562  //
//#define HEAD_LOC_X                       0.0767500000  //
//#define NECK_BASE_Z                      0.0617500000  // Axle to Neck
//#define NECK_BASE_X                      0.0767500000  // Axle to Neck
}

/*******************************************************************************
* Silvergoose IMU Update
*******************************************************************************/
void imu_update()
{
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;

  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    digitalWrite( led_pin, led_tog );
    led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];

  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();
    
#ifdef DEBUG_IMU
    Serial.print(imu_time);
    Serial.print(" ");
    Serial.print(IMU.rpy[0]);
    Serial.print(" ");
    Serial.print(IMU.rpy[1]);
    Serial.print(" ");
    Serial.println(IMU.rpy[2]);
#endif

  }

#ifdef DEBUG_IMU
  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
      }
      Serial.print("ACC Cali End ");
    }
  }
#endif
}

