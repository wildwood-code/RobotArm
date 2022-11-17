/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : ServoBus.h
  Class      : ServoBus
  Description:
    The ServoBus class encapsulates the serial write() and write_read() functions
    of the HiWonder/LewanSoul bus servos (LX-16A, LX-15D, LX-224, LX-225, LX-824,
    and LX-1501).

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once
#include <HardwareSerial.h>

#define BUS_BUFLEN  32

namespace Robotics
{
class ServoBus
{
  public:
    ServoBus(HardwareSerial &serial, uint8_t tx_ena_pin, uint16_t msec = 0);
    virtual ~ServoBus();
    ServoBus& operator = (ServoBus const&) = delete;

    void set_timeout(uint16_t msec);
    bool write(uint8_t id, uint8_t dlc, uint8_t cmd, uint8_t const* buf);
    bool write_read(uint8_t id, uint8_t& dlc, uint8_t cmd, uint8_t* buf, size_t buflen);
    operator bool() const;

  private:
    HardwareSerial *pserial;
    uint8_t        tx_ena;
    uint16_t       timeout;

    // timeout detection: millis() cycles after 50 days, so detect roll-over
    unsigned long time_stop;
    unsigned long time_start;
    bool time_rollover;

    void start_timer();
    bool is_timeout() const;
};
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
