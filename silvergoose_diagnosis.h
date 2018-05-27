#ifndef SILVERGOOSE_DIAGNOSIS_
#define SILVERGOOSE_DIAGNOSIS_

#include "silvergoose_defines.h"

uint8_t battery_state = 0;

#define LED_TXD                          0
#define LED_RXD                          1
#define LED_LOW_BATTERY                  2
#define LED_ROS_CONNECT                  3
#define LED_WORKING_CHECK                13

#define BATTERY_POWER_OFF                0
#define BATTERY_POWER_STARTUP            1
#define BATTERY_POWER_NORMAL             2
#define BATTERY_POWER_CHECK              3
#define BATTERY_POWER_WARNNING           4

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

void setPowerOn(void)
{
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
}

void setPowerOff(void)
{
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

uint8_t updateVoltageCheck(bool check_setup)
{  
  static uint8_t battery_voltage     = 0;
  static float   battery_valtage_raw = 0;
  static uint8_t battery_state       = BATTERY_POWER_OFF;

  static bool startup = false;
  static int vol_index = 0;
  static int prev_state = 0;
  static int alram_state = 0;
  static int check_index = 0;

  int i;
  float vol_sum;
  float vol_value;

  static uint32_t process_time[8] = {0,};
  static float    vol_value_tbl[10] = {0,};

  float voltage_ref       = 11.0 + 0.0;
  float voltage_ref_warn  = 11.0 + 0.0;


  if (startup == false)
  {
    startup = true;
    for (i=0; i<8; i++)
    {
      process_time[i] = millis();
    }
  }

  if (millis()-process_time[0] > 100)
  {
    process_time[0] = millis();

    vol_value_tbl[vol_index] = getPowerInVoltage();

    vol_index++;
    vol_index %= 10;

    vol_sum = 0;
    for(i=0; i<10; i++)
    {
      vol_sum += vol_value_tbl[i];
    }
    vol_value = vol_sum/10;
    battery_valtage_raw = vol_value;

    //Serial.println(vol_value);

    battery_voltage = vol_value;
  }


  if(millis()-process_time[1] > 1000)
  {
    process_time[1] = millis();

    //Serial.println(battery_state);

    switch(battery_state)
    {
      case BATTERY_POWER_OFF:
        if (check_setup == true)
        {
          alram_state = 0;
          if(battery_valtage_raw > 5.0)
          {
            check_index    = 0;
            prev_state     = battery_state;
            battery_state = BATTERY_POWER_STARTUP;
          }
          else
          {
            noTone(BDPIN_BUZZER);
          }
        }
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref)
        {
          check_index   = 0;
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
          setPowerOn();
        }

        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if (battery_valtage_raw > 5.0)
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_CHECK;
          }
          else
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_OFF;
          }
        }
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
          check_index   = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if(battery_valtage_raw < voltage_ref_warn)
          {
            setPowerOff();
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_WARNNING;
          }
        }
        if(battery_valtage_raw >= voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state)
        {
          tone(BDPIN_BUZZER, 1000, 500);
        }

        if(battery_valtage_raw > voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        else
        {
          setPowerOff();
        }

        if(battery_valtage_raw < 5.0)
        {
          setPowerOff();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_OFF;
        }
        break;

      default:
        break;
    }
  }

  return battery_state;
}

uint8_t getButtonPress(void)
{
  uint8_t button_state = 0;
  static uint32_t t_time[2];
  static uint8_t button_state_num[2] = {0, };

  for (int button_num = 0; button_num < 2; button_num++)
  {
    switch (button_state_num[button_num])
    {
     case WAIT_FOR_BUTTON_PRESS:
       if (getPushButton() & (1 << button_num))
       {
         t_time[button_num] = millis();
         button_state_num[button_num] = WAIT_SECOND;
       }
       break;

     case WAIT_SECOND:
       if ((millis()-t_time[button_num]) >= 1000)
       {
         if (getPushButton() & (1 << button_num))
         {
           button_state_num[button_num] = CHECK_BUTTON_RELEASED;
           button_state |= (1 << button_num);
         }
         else
         {
           button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
         }
       }
       break;

     case CHECK_BUTTON_RELEASED:
       if (!(getPushButton() & (1 << button_num)))
         button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
       break;

     default :
       button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
       break;
    }
  }

  return button_state;
}

#endif //SILVERGOOSE_DIAGNOSIS_

