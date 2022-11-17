/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : SensorGrid.cpp
  Class      : SensorGrid
  Description:
    X-Y grid of ultrasonic sensors to detect the position of an object

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>

#include "SensorGrid.h"


Robotics::SensorGrid::SensorGrid(int trig_x, int trig_y, unsigned long timeout)
: pin_trig_x(trig_x), pin_trig_y(trig_y), timeout(timeout), pEcho_x(NULL), pEcho_y(NULL)
{
  // configure TRIG pins
  pinMode(trig_x, OUTPUT);
  digitalWrite(trig_x, LOW);
  pinMode(trig_y, OUTPUT);
  digitalWrite(trig_y, LOW);
}


Robotics::SensorGrid::~SensorGrid()
{
  // TODO: delete pEcho_x and pEcho_y allocations
}

bool Robotics::SensorGrid::AddEchoX(int pin)
{
  SensorGrid::PinEntry *pEntry = AddEchoPin(pin, pEcho_x);

  if (pEntry!=NULL)
    pEcho_x = pEntry;

  return true;
}


bool Robotics::SensorGrid::AddEchoY(int pin)
{
  SensorGrid::PinEntry *pEntry = AddEchoPin(pin, pEcho_y);

  if (pEntry!=NULL)
    pEcho_y = pEntry;

  return true;
}


Robotics::SensorGrid::PinEntry* Robotics::SensorGrid::AddEchoPin(int pin, Robotics::SensorGrid::PinEntry* pEntry)
{
  if (pEntry==NULL)
  {
    pEntry = new SensorGrid::PinEntry;
    pEntry->pin = pin;
    pEntry->next = NULL;
    return pEntry;  // indicate that initial entry was created
  }
  else
  {
    while (pEntry->next!=NULL)
      pEntry = pEntry->next;
    pEntry->next = new SensorGrid::PinEntry;
    pEntry = pEntry->next;
    pEntry->pin = pin;
    pEntry->next = NULL;
    return NULL;  // indicate that initial entry was not created
  }
}


bool Robotics::SensorGrid::LocateRaw(unsigned long& x, unsigned long& y)
{
  bool result = false;

  unsigned long xx = 0;
  unsigned long yy = 0;

  SensorGrid::PinEntry *pEntry;
  
  // locate X
  pEntry = pEcho_x;
  while (pEntry!=NULL)
  {
    digitalWrite(pin_trig_x, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin_trig_x, LOW);

    unsigned long tp = pulseIn(pEntry->pin, HIGH);

    if (tp>0)
    {
      if (xx==0 || tp<xx)
        xx = tp;
    }

    pEntry = pEntry->next;
  }

  // locate Y
  pEntry = pEcho_y;
  while (pEntry!=NULL)
  {
    digitalWrite(pin_trig_y, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin_trig_y, LOW);

    unsigned long tp = pulseIn(pEntry->pin, HIGH);

    if (tp>0)
    {
      if (yy==0 || tp<yy)
        yy = tp;
    }

    pEntry = pEntry->next;
  }

  if (xx>0 && yy>0)
  {
    result = true;
    x = xx;
    y = yy;
  }

  return result;
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
