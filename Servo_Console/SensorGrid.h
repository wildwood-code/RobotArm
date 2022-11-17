/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : SensorGrid.h
  Class      : SensorGrid
  Description:
    X-Y grid of ultrasonic sensors to detect the position of an object

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once

#define TIMEOUT_DEFAULT   4000  // microseconds

namespace Robotics
{
class SensorGrid
{
  protected:
    unsigned long timeout;
    int pin_trig_x;
    int pin_trig_y;

    struct PinEntry
    {
      int pin;
      struct PinEntry *next;
    } *pEcho_x, *pEcho_y;

  public:
    SensorGrid(int trig_x, int trig_y, unsigned long timeout = TIMEOUT_DEFAULT);
    virtual ~SensorGrid();
    SensorGrid& operator = (SensorGrid const&) = delete;

    // Pin configuration
    bool AddEchoX(int pin);
    bool AddEchoY(int pin);

    // object detection
    bool LocateRaw(unsigned long& x, unsigned long& y);

  private:
    static PinEntry* AddEchoPin(int pin, PinEntry* pEntry);
};
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
