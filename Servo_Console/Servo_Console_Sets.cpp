/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Sets.cpp
  Description:
    Implements the functions for reading angle, storing it, and moving to stored
    angles for the set of all known servos that have been enumerated.

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>
#include <Regexp.h>

#include "Servo.h"
#include "Servo_Console_Support.h"
#include "Servo_Console_Extern.h"

// Size of the position set arrays
#define MAX_POSITION_SET_NUM  31
#define MAX_NUM_SET_SERVOS    6

// GOTO time algorithm - scaled from MIN to MAX using largest absolute position difference
#define GOTO_TIME_MAX    4000
#define GOTO_TIME_MIN    100
#define POS_DELTA_FULL_SCALE  240.0  // used for scaling move times

// Declare the position set arrays
//   position_id[] contains the IDs of the servos that are enumerated
//   position_set[][] contains each set of positions for each ID
Robotics::servo_id_t position_id[MAX_NUM_SET_SERVOS];
Robotics::servo_angle_t position_set[MAX_POSITION_SET_NUM + 1][MAX_NUM_SET_SERVOS];

// Initial positions pre-loaded into set 0 for specific IDs
const Robotics::servo_id_t ID_Initial[MAX_NUM_SET_SERVOS] = { 1, 2, 3, 4, 5, 6 };
const double Pos_Initial[MAX_NUM_SET_SERVOS] = { 40.0, 120.0, 120.0, 120.0, 120.0, 120.0 };

// pre-declare some functions
void Enumerate_Position_IDs();


/*******************************************************************************
  Function Definitions
*******************************************************************************/

void Initialize_Position_Sets()
{
  size_t idx_set;
  size_t idx_servo;

  // initialize all sets to default values
  for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
  {
    position_id[idx_servo] = ID_RESERVED;

    for (idx_set = 0; idx_set <= MAX_POSITION_SET_NUM; ++idx_set)
      position_set[idx_set][idx_servo] = INVALID_ANGLE;
  }

  // scan the servo bus to identify which IDs are present on the bus
  Enumerate_Position_IDs();

  // pre-load "home" positions (set 0) for matching IDs
  for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
  {
    size_t idx_default;
    Robotics::servo_id_t id = position_id[idx_servo];

    for (idx_default = 0; idx_default < MAX_NUM_SET_SERVOS; ++idx_default)
    {
      if (id == ID_Initial[idx_default])
      {
        position_set[0][idx_servo] = Pos_Initial[idx_default];
        break;
      }
    }
  }
}


void Enumerate_Position_IDs()
{ // only do this if it has not been completed
  if (position_id[0] == ID_RESERVED)
  {
    size_t idx_servo = 0; // next to assign
    Robotics::servo_id_t id;
    Robotics::servo_id_t old_id;

    old_id = servo;

    for (id = 0; id <= ID_MAX; ++id)
    {
      servo = id;
      if (servo.is_attached())
      {
        position_id[idx_servo++] = id;

        if (idx_servo >= MAX_NUM_SET_SERVOS)
          break; // position_id[] is full... stop here
      }
    }

    servo = old_id;
  }
}


void Save_Positions_to_Set(uint8_t set)
{
  if (position_id[0] != ID_RESERVED)
  {
    size_t idx_servo;
    Robotics::servo_id_t old_id;
    old_id = servo;

    for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
    {
      Robotics::servo_id_t id = position_id[idx_servo];

      if (id != ID_RESERVED)
      {
        servo = id;
        position_set[set][idx_servo] = servo.angle();
      }
      else
      {
        position_set[set][idx_servo] = INVALID_ANGLE;
      }
    }

    servo = old_id;
  }
}


void Goto_Positions_in_Set(uint8_t set)
{
  if (position_id[0] != ID_RESERVED)
  {
    size_t idx_servo;
    Robotics::servo_id_t old_id;
    old_id = servo;
    double max_abs_scaled_delta = 0.0;

    // take a snapshot of current positions and calculate the max normalized abs difference
    for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
    {
      Robotics::servo_id_t id = position_id[idx_servo];
      
      if (id != ID_RESERVED)
      {
        servo = id;

        // take a snapshot of current position and grab the new position
        double pos_old = servo.angle();
        double pos = position_set[set][idx_servo];

        // calculate the absolute delta, save the maximum
        double pos_delta;
        if (pos != INVALID_ANGLE && pos_old != INVALID_ANGLE)
          pos_delta = pos - pos_old;
        else
          pos_delta = POS_DELTA_FULL_SCALE;

        // take absolute value, limit to <= 1.0
        if (pos_delta < 0.0)
          pos_delta = -pos_delta;


        pos_delta = pos_delta / POS_DELTA_FULL_SCALE;   // normalize

        if (pos_delta > 1.0)
          pos_delta = 1.0;

        if (pos_delta > max_abs_scaled_delta)
          max_abs_scaled_delta = pos_delta;
      }
    }

    // calculate delay time based upon max delta
    uint16_t goto_time = GOTO_TIME_MIN + uint16_t(double(GOTO_TIME_MAX-GOTO_TIME_MIN)*max_abs_scaled_delta); 
    
    for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
    {
      Robotics::servo_id_t id = position_id[idx_servo];

      if (id != ID_RESERVED)
      {
        servo = id;

        // configure new positions for move with wait
        double pos = position_set[set][idx_servo];

        if (pos != INVALID_ANGLE)
          servo.move_wait(pos, goto_time);
      }
    }

    // start all servos to the new positions
    broadcast.start();

    // restore the original ID
    servo = old_id;
  }
}


bool Are_Position_IDs_Enumerated()
{
  if (position_id[0] == ID_RESERVED)
    return false;
  else
    return true;
}


void Dump_Position_Set_Header()
{
  char buf[CHAR_BUF_LEN];
  size_t idx_servo;
  Serial.print(F("SET  "));
  for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
  {
    Robotics::servo_id_t id = position_id[idx_servo];

    if (id == ID_RESERVED)
    {
      Serial.print(F("    NONE"));
    }
    else
    {
      sprintf(buf, "   ID=%02d", id);
      Serial.print(buf);
    }
  }
  Serial.println();
}


void Print_Invalid_Angle()
{
  Serial.print(F("  ------"));
}


void Dump_Position_Set(uint8_t set, bool header)
{
  char buf[CHAR_BUF_LEN];
  size_t idx_servo;

  if (header)
    Dump_Position_Set_Header();

  sprintf(buf, "%3d  ", set);
  Serial.print(buf);

  for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
  {
    Robotics::servo_id_t id = position_id[idx_servo];

    if (id == ID_RESERVED)
    {
      Print_Invalid_Angle();
    }
    else
    {
      double pos = position_set[set][idx_servo];

      if (pos == INVALID_ANGLE)
      {
        Print_Invalid_Angle();
      }
      else
      {
        dtostrf(pos, 8, 1, buf);
        Serial.print(buf);
      }
    }
  }

  Serial.println();
}


void Dump_Position_Sets()
{
  bool dumped_one = false;
  uint8_t set;

  Dump_Position_Set_Header();

  for (set = 0; set <= MAX_POSITION_SET_NUM; ++set)
  {
    if (position_set[set][0] != INVALID_ANGLE)
    {
      Dump_Position_Set(set, false);
      dumped_one = true;
    }
  }

  if (!dumped_one)
    Serial.println(F("No sets defined"));
}


void Dump_Position_Servos()
{
  char buf[CHAR_BUF_LEN];
  Enumerate_Position_IDs();

  if (position_id[0] != ID_RESERVED)
  {
    size_t idx_servo;
    Robotics::servo_id_t old_id;
    old_id = servo;

    Serial.print(F("\n"));

    for (idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
    {
      Robotics::servo_id_t id = position_id[idx_servo];

      if (id != ID_RESERVED)
      {
        servo = id;
        double pos = servo.angle();
        sprintf(buf, "  %2d = ", id);
        Serial.print(buf);
        if (pos == INVALID_ANGLE)
        {
          Print_Invalid_Angle();
        }
        else
        {
          dtostrf(pos, 8, 1, buf);
          Serial.print(buf);
        }

        Serial.print(F("\n"));
      }
    }

    servo = old_id;
  }
  else
  {
    Serial.print(F("No servos\n"));
  }
}


void Save_Positions(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0); // get the set #
  uint8_t set = atoi(buf);

  if (set > MAX_POSITION_SET_NUM)
    ServoConsole_syntax_error(); // TODO: better error reporting

  Save_Positions_to_Set(set);
}


void Goto_Positions(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0); // get the set #
  uint8_t set = atoi(buf);

  if (set > MAX_POSITION_SET_NUM)
  {
    ServoConsole_syntax_error(); // TODO: better error reporting
  }
  else
  {
    if (!Are_Position_IDs_Enumerated())
      ServoConsole_syntax_error();

    Goto_Positions_in_Set(set);
  }
}


void Goto_Home()
{
  Goto_Positions_in_Set(0);
}


void Dump_Positions(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];
  char args[CHAR_BUF_LEN];
  MatchState ma;

  ms.GetCapture(buf, 0);
  ma.Target(buf);

  if (ma.Match("^(%d+)$"))
  {
    ma.GetCapture(args, 0); // get the set #
    uint8_t set = atoi(args);
    Dump_Position_Set(set, true);
  }
  else if (ma.Match("^SE?R?V?O?S?$"))
  {
    Dump_Position_Servos();
  }
  else
  {
    ServoConsole_syntax_error();
  }
}


void Dump_Positions_All()
{
  Dump_Position_Sets();
}


void Set_Positions(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];
  char args[CHAR_BUF_LEN];
  MatchState ma;
  size_t idx_servo = 0;
  unsigned int match_loc = 0;

  ms.GetCapture(buf, 0);
  uint8_t set = atoi(buf);
  ms.GetCapture(buf, 1);
  ma.Target(buf);

  while (idx_servo < MAX_NUM_SET_SERVOS)
  {
    char const szRe[] = "^%s*(%d+)%s*";
    if (ma.Match(szRe, match_loc))
    {
      ma.GetCapture(args, 0);
      position_set[set][idx_servo++] = atof(args);
      match_loc = match_loc + ma.MatchLength;
    }
    else
    {
      break;
    }
  }
}


bool Get_Positions(uint8_t set)
{
  bool bResult = false;
  char buf[CHAR_BUF_LEN];

  if (position_set[set][0] != INVALID_ANGLE)
  {
    sprintf(buf, "SET %d =", set);
    Serial.print(buf);

    for (size_t idx_servo = 0; idx_servo < MAX_NUM_SET_SERVOS; ++idx_servo)
    {
      double pos = position_set[set][idx_servo];

      if (pos == INVALID_ANGLE)
        break;

      sprintf(buf, " %d", int(pos + 0.5)); // round it to nearest integer
      Serial.print(buf);
    }

    Serial.println();
    bResult = true;
  }

  return bResult;
}


void Get_Positions(MatchState& ms)
{
  char buf[CHAR_BUF_LEN];

  ms.GetCapture(buf, 0);
  uint8_t set = atoi(buf);
  if (set <= MAX_POSITION_SET_NUM)
    Get_Positions(set);
}


void Get_Positions()
{
  size_t idx_set;

  for (idx_set = 0; idx_set <= MAX_POSITION_SET_NUM; ++idx_set)
    Get_Positions(idx_set);
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
