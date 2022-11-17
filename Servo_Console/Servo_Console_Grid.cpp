/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Grid.cpp
  Description:
    Implements the functions for detecting object location on the sensor grid.

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>
#include <Regexp.h>

#include "Servo_Console_Support.h"
#include "SensorGrid.h"


#define DIST_X_TRIG_PIN   22
#define DIST_X_ECHO1_PIN  24
#define DIST_X_ECHO2_PIN  26
#define DIST_X_ECHO3_PIN  28
#define DIST_X_ECHO4_PIN  30

#define DIST_Y_TRIG_PIN   23
#define DIST_Y_ECHO1_PIN  25
#define DIST_Y_ECHO2_PIN  27
#define DIST_Y_ECHO3_PIN  29
#define DIST_Y_ECHO4_PIN  31


Robotics::SensorGrid grid(DIST_X_TRIG_PIN, DIST_Y_TRIG_PIN);


void Initialize_Grid()
{ // build the grid of X and Y ultrasonic sensors
  grid.AddEchoX(DIST_X_ECHO1_PIN);
  grid.AddEchoX(DIST_X_ECHO2_PIN);
  grid.AddEchoX(DIST_X_ECHO3_PIN);
  grid.AddEchoX(DIST_X_ECHO4_PIN);
  grid.AddEchoY(DIST_Y_ECHO1_PIN);
  grid.AddEchoY(DIST_Y_ECHO2_PIN);
  grid.AddEchoY(DIST_Y_ECHO3_PIN);
  grid.AddEchoY(DIST_Y_ECHO4_PIN);  
}


void Distance_Query()
{
  unsigned long x, y;

  if (grid.LocateRaw(x, y))
  {
    ServoConsole_report(x);
    Serial.print(F(" "));
    Serial.print(y);
  }
  else
  {
    ServoConsole_report(F("NO OBJECT"));
  }
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
