/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Servos.cpp
  Description:
    Implements the functions for controlling the servos via the console.

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>
#include <Regexp.h>

#include "Servo_Console_Support.h"
#include "Servo.h"


#define BUS_PORT Serial3
#define BUS_TX_CTRL 2
#define BUF_LEN  16
#define SERVOBUS_TIMEOUT  20    // msec


Robotics::ServoBus bus(BUS_PORT, BUS_TX_CTRL, SERVOBUS_TIMEOUT);
Robotics::Servo broadcast(ID_BROADCAST, &bus);
Robotics::Servo servo(ID_RESERVED, &bus);


void Initialize_Servos()
{
}


void Angle_Query()
{
  ServoConsole_report(servo.angle());
}


void Vin_Query()
{
  ServoConsole_report(servo.V());
}


void Temp_Query()
{
  ServoConsole_report(servo.T());
}


void Offset_Query()
{
  ServoConsole_report(int(servo.offset()));
}


void ID_Query()
{
  ServoConsole_report(int(Robotics::servo_id_t(servo)));
}


void Limit_Temp(MatchState& ms)
{
  bool result = false;
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  Robotics::servo_temp_t max = atof(buf);
  ms.GetCapture(buf, 1);

  if (strlen(buf) > 0)
    result = broadcast.set_limit_T(max);
  else
    result = servo.set_limit_T(max);

  if (!result)
    ServoConsole_report_failure();
}




void Limit_Temp_Query()
{
  Robotics::servo_temp_t max = INVALID_TEMP;
  if (servo.get_limit_T(max))
    ServoConsole_report(max);
  else
    ServoConsole_report_failure();
}


void Limit_Vin_Query()
{
  Robotics::servo_temp_t max = INVALID_TEMP;
  Robotics::servo_temp_t min = INVALID_TEMP;

  if (servo.get_limit_V(min, max))
  {
    ServoConsole_report(min);
    Serial.print(F(" "));
    Serial.print(max);
  }
  else
  {
    ServoConsole_report_failure();
  }
}


void Limit_Angle_Query()
{
  Robotics::servo_temp_t max = INVALID_TEMP;
  Robotics::servo_temp_t min = INVALID_TEMP;

  if (servo.get_limit(min, max))
  {
    ServoConsole_report(min);
    Serial.print(F(" "));
    Serial.print(max);
  }
  else
  {
    ServoConsole_report_failure();
  }
}


void Limit_Vin(MatchState& ms)
{
  bool result = false;
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  Robotics::servo_voltage_t min = atof(buf);
  ms.GetCapture(buf, 1);
  Robotics::servo_voltage_t max = atof(buf);
  ms.GetCapture(buf, 2);

  if (strlen(buf) > 0)
    result = broadcast.set_limit_V(min, max);
  else
    result = servo.set_limit_V(min, max);

  if (!result)
    ServoConsole_report_failure();
}


void Limit_Angle(MatchState& ms)
{
  bool result = false;
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  Robotics::servo_angle_t min = atof(buf);
  ms.GetCapture(buf, 1);
  Robotics::servo_angle_t max = atof(buf);
  ms.GetCapture(buf, 2);

  if (strlen(buf) > 0)
    result = broadcast.set_limit(min, max);
  else
    result = servo.set_limit(min, max);

  if (!result)
    ServoConsole_report_failure();
}


void Change_ID(MatchState& ms)
{
  bool bresult = false;
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  Robotics::servo_id_t cur_id = atoi(buf);
  ms.GetCapture(buf, 1);
  Robotics::servo_id_t new_id = atoi(buf);

  if (cur_id == Robotics::servo_id_t(servo) && new_id >= ID_MIN && new_id <= ID_MAX)
  {
    uint16_t code = servo.unlock_id();
    bresult = servo.change_id(new_id, code);
  }

  if (bresult)
    ServoConsole_report_success();
  else
    ServoConsole_report_failure();
}


void Mode_Servo()
{
  if (!servo.set_mode(false))
    ServoConsole_report_failure();
}


void Mode_Servo_Broadcast()
{
  if (!broadcast.set_mode(false))
    ServoConsole_report_failure();
}


void Mode_Motor(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  int speed = atoi(buf);
  if (!servo.set_mode(true, speed))
    ServoConsole_report_failure();
}


void Mode_Query()
{
  bool motor;
  int speed;

  if (servo.get_mode(motor, speed))
  {
    if (motor)
    {
      ServoConsole_report(F("MOTOR "));
      Serial.print(speed);
    }
    else
    {
      ServoConsole_report(F("SERVO"));
    }
  }
  else
  {
    ServoConsole_report_failure();
  }
}


void Angle_Offset(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  int offs = atoi(buf);
  if (!servo.offset(offs))
    ServoConsole_report_failure();
}


void Angle_Offset_Write(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  int offs = atoi(buf);
  if (!servo.offset(offs, true))
    ServoConsole_report_failure();
}


void Angle_Offset_Write_Current()
{
  if (!servo.offset_write())
    ServoConsole_report_failure();
}


void Angle_Offset_Write_Current_Broadcast()
{
  if (!broadcast.offset_write())
    ServoConsole_report_failure();
}


void LED_Query()
{
  if (servo.LED())
    ServoConsole_report(F("ON"));
  else
    ServoConsole_report(F("OFF"));
}


void LED_On(MatchState& ms)
{ // no output generated for LED on/off
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  if (strlen(buf) > 0)
    broadcast.LED(true);
  else
    servo.LED(true);
}


void LED_Off(MatchState& ms)
{ // no output generated for LED on/off
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  if (strlen(buf) > 0)
    broadcast.LED(false);
  else
    servo.LED(false);
}


void LED_Error_Query()
{
  uint8_t fault = servo.LED_error();
  if (fault != INVALID_LED_FAULT)
    ServoConsole_report(int(fault));
  else
    ServoConsole_report_failure();
}


void LED_Error(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  uint8_t fault = atoi(buf);
  ms.GetCapture(buf, 1);

  if (strlen(buf) > 0)
    broadcast.LED_error(fault);
  else
    servo.LED_error(fault);
}


void Set_Active_ID(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  Robotics::servo_id_t id;
  ms.GetCapture(buf, 0);
  id = atoi(buf);
  servo = id;
}


void Enumerate_IDs()
{
  char buf[CHAR_BUF_LEN];

  Robotics::servo_id_t id;
  Robotics::servo_id_t last_id = ID_RESERVED;

  ServoConsole_report(F("querying IDs... "));
  for (id = 0; id <= ID_MAX; ++id)
  {
    servo = id;
    if (servo.is_attached())
    {
      if (last_id == ID_RESERVED)
        Serial.print(F("found ID"));

      sprintf(buf, " %d", id);
      Serial.print(buf);

      last_id = id;
    }
  }

  if (last_id == ID_RESERVED)
    Serial.print(F("none!"));
  else
    Serial.print(F(" ... done."));

  servo = last_id;
}

void Move_Start_Broadcast()
{
  if (!broadcast.start())
    ServoConsole_report_failure();
}


void Move_Start()
{
  if (!servo.start())
    ServoConsole_report_failure();
}


void Move_Stop_Broadcast()
{
  if (!broadcast.stop())
    ServoConsole_report_failure();
}


void Move_Stop()
{
  if (!servo.stop())
    ServoConsole_report_failure();
}


void Load_Servos_Broadcast()
{
  if (!broadcast.load())
    ServoConsole_report_failure();
}


void Load_Servo()
{
  if (!servo.load())
    ServoConsole_report_failure();
}


void Unload_Servos_Broadcast()
{
  if (!broadcast.unload())
    ServoConsole_report_failure();
}


void Unload_Servo()
{
  if (!servo.unload())
    ServoConsole_report_failure();
}


void Move_Servos(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];
  char args[CHAR_BUF_LEN];
  MatchState ma;

  bool move_wait = false;
  bool move_broadcast = false;
  int move_time = 0;

  ms.GetCapture(buf, 0); // get the position
  int pos = atoi(buf);
  ms.GetCapture(args, 1); // get the rest

  ma.Target(args);
  if (ma.Match("(WA?I?T?)"))
    move_wait = true;
  if (ma.Match("(%*)"))
    move_broadcast = true;
  if (ma.Match("(%d+)"))
  {
    ma.GetCapture(buf, 0);
    move_time = atoi(buf);
  }

  if (move_wait)
  {
    if (move_broadcast)
      broadcast.move_wait(pos, move_time);
    else
      servo.move_wait(pos, move_time);
  }
  else
  {
    if (move_broadcast)
      broadcast.move(pos, move_time);
    else
      servo.move(pos, move_time);
  }
}


void Servo_Report(uint8_t id)
{
  Robotics::Servo_Report report;
  Robotics::servo_id_t save_id = servo;
  char buf[CHAR_BUF_LEN];
  uint16_t d_count;

  servo = id;

  if (servo.report(report))
  {
    sprintf(buf, "ID = %d\n", report.ID);
    Serial.print(buf);

    if (report.IS_MOTOR)
    {
      Serial.print(F("  MOTOR\n  SPEED = "));
      Serial.println(report.MOTOR.SPEED);
    }
    else
    {
      Serial.print(F("  SERVO\n  ANGLE = "));
      Serial.print(report.SERVO.A);
      Serial.print(F("  ["));
      Serial.print(report.SERVO.A_min);
      Serial.print(F(","));
      Serial.print(report.SERVO.A_max);
      Serial.print(F("]\n  "));
      if (!report.SERVO.IS_LOADED)
        Serial.print(F("UN"));
      Serial.println(F("LOADED"));
      Serial.print(F("  OFFSET = "));
      Serial.println(int(report.SERVO.OFFSET));
    }

    Serial.print(F("  VOLTAGE = "));
    Serial.print(report.V);
    Serial.print(F("  ["));
    Serial.print(report.V_min);
    Serial.print(F(","));
    Serial.print(report.V_max);
    Serial.println(F("]"));
    Serial.print(F("  TEMP = "));
    Serial.print(report.T);
    Serial.print(F("  ["));
    Serial.print(report.T_max);
    Serial.println(F("]"));
    Serial.print(F("  LED = "));
    if (report.LED)
      Serial.println(F("ON"));
    else
      Serial.println(F("OFF"));
    Serial.print(F("  LED ALARM MODE = "));
    Serial.println(int(report.LED_error));
  }

  servo = save_id;
}


void Servo_Report()
{
  for (Robotics::servo_id_t id = 0; id <= ID_MAX; ++id)
    Servo_Report(id);
}


void Servo_Report(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0); // get the set #
  int id = atoi(buf);

  if (id >= 0 && id <= ID_MAX)
    Servo_Report(id);
}

/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
