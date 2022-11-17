/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : ServoBus.cpp
  Class      : ServoBus
  Description:
    The ServoBus class encapsulates the serial write() and write_read() functions
    of the HiWonder/LewanSoul bus servos (LX-16A, LX-15D, LX-224, LX-225, LX-824,
    and LX-1501).

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>
#include "ServoBus.h"

#define DLC_MIN 3
#define DLC_MAX 7

//#define SERVOBUS_DEBUG    // comment-out to disable


void Robotics::ServoBus::start_timer()
{
  time_start = millis();
  time_stop = time_start + timeout;
  time_rollover = time_stop < time_start ? true : false;
}


bool Robotics::ServoBus::is_timeout() const
{
  unsigned long time_now = millis();

  if (time_now >= time_stop && (time_rollover == false || time_now < time_start))
    return true;
  else
    return false;
}


// Timeout is specified in msec. If msec==0, then the constructor will not
// wait for serial initialization to complete
Robotics::ServoBus::ServoBus(HardwareSerial &serial, uint8_t tx_ena_pin, uint16_t msec)
  : pserial(&serial), tx_ena(tx_ena_pin), timeout(msec)
{
  start_timer();

  pinMode(tx_ena, OUTPUT);
  digitalWrite(tx_ena, LOW); // initially disable TX

  pserial->begin(115200, SERIAL_8N1);

  if (timeout > 0)
  { // wait for serial to complete initialization, or for timeout to occur
    for (;;)
    {
      if (*pserial)
      { // initialization complete
        break;
      }
      else if (is_timeout())
      { // timeout has occurred, nullify pointer and fail out
        pserial = NULL;
        break;
      }
    }
  }
}


Robotics::ServoBus::~ServoBus()
{
  pserial->end();
}


void Robotics::ServoBus::set_timeout(uint16_t msec)
{
  timeout = msec;
}


Robotics::ServoBus::operator bool() const
{
  if (pserial)  return (*pserial);
  else          return false;
}


bool Robotics::ServoBus::write(uint8_t id, uint8_t dlc, uint8_t cmd, uint8_t const* buf)
{
  bool bresult = false;
  uint8_t hs_buf[BUS_BUFLEN];

  start_timer();

  if (pserial && (dlc >= DLC_MIN && dlc <= DLC_MAX))
  {
    size_t i;
    uint8_t cks = 0;

    // Build the packet
    hs_buf[0] = 0x55;
    hs_buf[1] = 0x55;
    hs_buf[2] = id;
    hs_buf[3] = dlc;
    hs_buf[4] = cmd;
    for (i = 0; i < dlc - 3; ++i)    hs_buf[5 + i] = buf[i];
    for (i = 2; i <= dlc + 1; ++i)   cks += hs_buf[i];
    hs_buf[dlc + 2] = ~cks;

#ifdef SERVOBUS_DEBUG
    Serial.print("WRITE: ");
    size_t idebug;
    for (idebug = 0; idebug <= dlc + 2; ++idebug)
    {
      char szbuf[128];
      sprintf(szbuf, "%02X ", hs_buf[idebug]);
      Serial.print(szbuf);

    }
    Serial.print("\n");
#endif

    // complete in-progress TX and RX operations
    pserial->flush();
    while (pserial->available() > 0)
      pserial->read();  // discard any RX data

    // enable transmission to the bus, then transmit the message
    digitalWrite(tx_ena, HIGH);
    delayMicroseconds(10);
    pserial->write(hs_buf, dlc + 3);

    i = 0;  // start of buffer
    while (timeout == 0 || !is_timeout())
    {
      if (pserial->available() > 0)
      {
        uint8_t r = pserial->read();
        if (r == hs_buf[i])
        {
          if (++i >= dlc + 3)
          { // matched the whole transmit buffer... success!
            bresult = true;
            break;
          }
        }
        else
        { // data did not match hs_buf (old data or corrupted?)
          // start back at beginning of hs_buf
          i = 0;
        }
      }
    }

    // disable transmit
    digitalWrite(tx_ena, LOW);
  }

  return bresult;
}


bool Robotics::ServoBus::write_read(uint8_t id, uint8_t& dlc, uint8_t cmd, uint8_t* buf, size_t buflen)
{
  // first, transmit the package (this starts the timeout timer)
  bool bresult = write(id, dlc, cmd, buf);

  if (bresult)
  { // now, wait for a response package which must have same id and cmd (dlc may differ)
    uint8_t cks = 0;
    size_t i = 0;
    size_t ibuf = 0;

    bresult = false;
    while (timeout == 0 || !is_timeout())
    {
      if (pserial->available() > 0)
      {
        uint8_t rd = pserial->read();

        if (i < 2)
        { // header field = 0x55
          if (rd != 0x55)
            break;
          i += 1;
        }
        else if (i == 2)
        { // ID field must match
          if (rd != id)
            break;
          i += 1;
          cks += rd;
        }
        else if (i == 3)
        { // DLC field must be within valid range, save it
          dlc = rd;
          if (dlc < DLC_MIN || dlc > DLC_MAX)
            break;
          i += 1;
          cks += rd;
        }
        else if (i == 4)
        { // CMD field must match
          if (rd != cmd)
            break;
          i += 1;
          cks += rd;
        }
        else if (i < dlc + 2)
        { // store the data field
          if (ibuf < buflen)
            buf[ibuf++] = rd;
          i += 1;
          cks += rd;
        }
        else
        { // CKS field: received must equal calculated
          if ((rd + cks) & 0xFF == 0xFF)
            bresult = true;
          break;
        }
      }
    }
  }

  return bresult;
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
