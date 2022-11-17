/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo.cpp
  Class      : Servo
  Description:
    The Servo class encapsulates the movement and control functions of the
    HiWonder/LewanSoul bus servos (LX-16A, LX-15D, LX-224, LX-225, LX-824,
    and LX-1501).

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#include <Arduino.h>
#include "Servo.h"

#define RAW_MIN  0
#define RAW_MAX  1000
#define DEG_MIN  0.0
#define DEG_MAX  240.0
#define MSEC_MIN 0
#define MSEC_MAX 30000
#define MV_SCALE  1000
#define TOL_RAW_MIN   0
#define TOL_RAW_MAX   21   // 5.04 deg

#define RAW_ANG_TO_DEG(raw) (DEG_MIN+(DEG_MAX-DEG_MIN)*double(raw-RAW_MIN)/(RAW_MAX-RAW_MIN))
#define DEG_TO_RAW_ANG(deg) (RAW_MIN+uint16_t((RAW_MAX-RAW_MIN)*(deg-DEG_MIN)/(DEG_MAX-DEG_MIN)+0.5))   // nearest raw angle
#define COERCE_RAW_ANG(raw) ( (raw)<RAW_MIN ? RAW_MIN : ( (raw)>RAW_MAX ? RAW_MAX : (raw) ) )
#define CHECK_RAW_ANG(raw)  ( (raw)>=RAW_MIN && (raw)<=RAW_MAX )
#define CHECK_MSEC(msec)    ( (msec)>=MSEC_MIN && (msec)<=MSEC_MAX )
#define UINT16_UPPER(ui)    (uint8_t(ui>>8))
#define UINT16_LOWER(ui)    (uint8_t(ui&0x00FF))
#define UINT16_BUILD(up,lo) (uint16_t(up)*256 + uint16_t(lo))
#define RAW_V_TO_VOLTS(raw) (servo_voltage_t(raw)/MV_SCALE)
#define VOLTS_TO_RAW_V(v)   (servo_voltage_raw_t(v*MV_SCALE))
#define RAW_T_TO_DEGC(raw)  (servo_temp_t(raw))
#define DEGC_TO_RAW_T(deg)  (servo_temp_raw_t(deg))


// ----------------------------------
// Servo construction and destruction
// ----------------------------------

Robotics::Servo::Servo(servo_id_t id, ServoBus* pbus)
  : id(id), pbus(pbus), tgt_angle(DEG_TO_RAW_ANG(INVALID_ANGLE)), tol_angle(DEG_TO_RAW_ANG(1.5))
{
}


Robotics::Servo::~Servo()
{
  detach();
}


// -------------------------
// Servo ServoBus connection
// -------------------------

bool Robotics::Servo::attach(ServoBus &bus)
{
  detach();
  pbus = &bus;
  return true;
}


bool Robotics::Servo::detach()
{
  pbus = NULL;
  return true;
}


bool Robotics::Servo::is_attached()
{
  bool result = false;
  uint8_t dlc = 3;

  // just send a get-position command and check to see if it responds
  if (command_response(0x1C, dlc))
    result = true;

  return result;
}


// --------------------
// Servo Motion Control
// --------------------

int Robotics::Servo::offset()
{
  int result = INVALID_OFFSET;
  uint8_t dlc = 3;

  if (command_response(0x13, dlc))
  {
    if (dlc == 4)
    {
      signed char ioffs = servo_buf[0];
      result = int(ioffs);
    }
  }

  return result;
}


bool Robotics::Servo::offset(int offs, bool write)
{
  bool result = false;
  servo_buf[0] = uint8_t(int8_t(offs));

  if (command(0x11, 4))
  {
    if (write)
      result = Robotics::Servo::offset_write();
    else
      result = true;
  }

  return result;
}


bool Robotics::Servo::offset_write()
{
  bool result = false;

  if (command(0x12, 3))
    result = true;

  return result;
}


bool Robotics::Servo::set_mode(bool motor, servo_speed_t speed)
{
  bool result = false;

  if (motor)
  { // motor mode
    servo_buf[0] = 1;
    servo_buf[1] = 0;
    servo_buf[2] = UINT16_LOWER(uint16_t(speed));
    servo_buf[3] = UINT16_UPPER(uint16_t(speed));
  }
  else
  { // servo mode
    servo_buf[0] = 0;
    servo_buf[1] = 0;
    servo_buf[2] = 0;
    servo_buf[3] = 0;
  }

  if (command(0x1D, 7))
    result = true;

  return result;
}


bool Robotics::Servo::get_mode(bool& motor, servo_speed_t& speed)
{
  bool result = false;
  uint8_t dlc = 3;

  if (command_response(0x1E, dlc))
  {
    if (dlc == 7)
    {
      motor = servo_buf[0] ? true : false;
      if (motor)
        speed = servo_speed_t(UINT16_BUILD(servo_buf[3], servo_buf[2]));
      else
        speed = 0;

      result = true;
    }
  }

  return result;
}


bool Robotics::Servo::speed(servo_speed_t speed)
{
  return Robotics::Servo::set_mode(true, speed);
}


Robotics::servo_speed_t Robotics::Servo::speed()
{
  bool motor;
  servo_speed_t speed = INVALID_SPEED;

  Robotics::Servo::get_mode(motor, speed);

  return speed;
}


bool Robotics::Servo::is_servo()
{
  bool motor;
  int speed = INVALID_SPEED;

  Robotics::Servo::get_mode(motor, speed);

  return !motor;
}


bool Robotics::Servo::is_motor()
{
  bool motor;
  int speed = INVALID_SPEED;

  Robotics::Servo::get_mode(motor, speed);

  return motor;
}


// get angular position
double Robotics::Servo::angle()
{
  double result = INVALID_ANGLE;
  uint8_t dlc = 3;

  if (command_response(0x1C, dlc))
  {
    if (dlc == 5)
      result = RAW_ANG_TO_DEG(int(UINT16_BUILD(servo_buf[1], servo_buf[0]))); // TODO: delete uint16_t(servo_buf[0])<<8 + uint16_t(servo_buf[1]);
  }

  return result;
}


// set position and time
double Robotics::Servo::move(double deg, uint16_t msec)
{
  double result = INVALID_ANGLE;

  uint16_t raw_angle = DEG_TO_RAW_ANG(deg);

  if (CHECK_MSEC(msec) && CHECK_RAW_ANG(raw_angle) )
  {
    tgt_angle = raw_angle;
    servo_buf[0] = UINT16_LOWER(raw_angle);
    servo_buf[1] = UINT16_UPPER(raw_angle);
    servo_buf[2] = UINT16_LOWER(msec);
    servo_buf[3] = UINT16_UPPER(msec);

    if (command(0x01, 7))
      result = deg;
  }

  return result;
}


double Robotics::Servo::move_wait(double deg, uint16_t msec)
{
  double result = INVALID_ANGLE;

  uint16_t raw_angle = DEG_TO_RAW_ANG(deg);

  if ( CHECK_MSEC(msec) && CHECK_RAW_ANG(raw_angle) )
  {
    tgt_angle = raw_angle;
    servo_buf[0] = UINT16_LOWER(raw_angle);
    servo_buf[1] = UINT16_UPPER(raw_angle);
    servo_buf[2] = UINT16_LOWER(msec);
    servo_buf[3] = UINT16_UPPER(msec);

    if (command(0x07, 7))
      result = deg;
  }

  return result;
}


bool Robotics::Servo::start()
{
  bool bresult = false;

  if (command(0x0B, 3))
    bresult = true;

  return bresult;
}


bool Robotics::Servo::stop()
{
  bool bresult = false;

  if (command(0x0C, 3))
    bresult = true;

  return bresult;
}


double Robotics::Servo::tol() const
{
  return RAW_ANG_TO_DEG(tol_angle);
}


double Robotics::Servo::tol(double ang)
{
  double result = INVALID_ANGLE;
  uint16_t raw_tol = DEG_TO_RAW_ANG(ang);

  if (raw_tol >= TOL_RAW_MIN && raw_tol <= TOL_RAW_MAX)
  {
    tol_angle = raw_tol;
    result = RAW_ANG_TO_DEG(tol_angle);
  }

  return result;
}


bool Robotics::Servo::is_move_complete()
{
  double bresult = false;
  uint8_t dlc = 3;

  if (command_response(0x1C, dlc))
  {
    if (dlc == 5)
    {
      uint16_t raw_ang = UINT16_BUILD(servo_buf[1], servo_buf[0]);
      if (raw_ang >= tgt_angle - tol_angle && raw_ang <= tgt_angle + tol_angle)
        bresult = true;
    }
  }

  return bresult;
}


bool Robotics::Servo::get_load(bool& load)
{
  bool bresult = false;
  uint8_t dlc = 3;

  if (command_response(0x20, dlc))
  {
    if (dlc == 4)
    {
      load = servo_buf[0] ? true : false;
      bresult = true;
    }
  }


  return bresult;
}

bool Robotics::Servo::load(bool on)
{
  bool bresult = false;

  if (on)
    servo_buf[0] = 1;
  else
    servo_buf[0] = 0;
  if (command(0x1F, 4))
    bresult = true;

  return bresult;
}


bool Robotics::Servo::unload(bool off)
{
  bool bresult = false;

  if (off)
    servo_buf[0] = 0;
  else
    servo_buf[0] = 1;
  if (command(0x1F, 4))
    bresult = true;

  return bresult;
}


// -----------------------------
// Servo Voltage and Temperature
// -----------------------------

double Robotics::Servo::V()
{
  double result = INVALID_VOLTAGE;
  uint8_t dlc = 3;

  if (command_response(0x1B, dlc))
  {
    if (dlc == 5)
      result = RAW_V_TO_VOLTS(UINT16_BUILD(servo_buf[1], servo_buf[0]));
  }

  return result;
}


double Robotics::Servo::T()
{
  double result = INVALID_TEMP;
  uint8_t dlc = 3;

  if (command_response(0x1A, dlc))
  {
    if (dlc == 4)
      result = RAW_T_TO_DEGC(servo_buf[0]);
  }

  return result;
}


// -----------
// LED Control
// -----------

bool Robotics::Servo::LED()
{
  bool result;
  uint8_t dlc = 3;

  if (command_response(0x22, dlc))
  {
    if (dlc == 4)
      result = servo_buf[0] ? false : true;
  }

  return result;
}


bool Robotics::Servo::LED(bool led)
{
  bool result;
  uint8_t dlc = 4;
  servo_buf[0] = led ? 0 : 1;
  result = command(0x21, dlc);

  return result;
}


bool Robotics::Servo::LED_error(uint8_t fault)
{
  bool result = false;

  if (fault >= 0 && fault <= 7)
  {
    uint8_t dlc = 4;
    servo_buf[0] = fault;
    result = command(0x23, dlc);
  }

  return result;
}


uint8_t Robotics::Servo::LED_error()
{
  uint8_t result = INVALID_LED_FAULT;
  uint8_t dlc = 3;

  if (command_response(0x24, dlc))
  {
    if (dlc == 4)
      result = servo_buf[0];
  }

  return result;
}


bool Robotics::Servo::set_limit(double min, double max)
{
  bool result = false;
  uint16_t pmin = DEG_TO_RAW_ANG(min);
  uint16_t pmax = DEG_TO_RAW_ANG(max);

  servo_buf[0] = UINT16_LOWER(pmin);
  servo_buf[1] = UINT16_UPPER(pmin);
  servo_buf[2] = UINT16_LOWER(pmax);
  servo_buf[3] = UINT16_UPPER(pmax);

  result = command(0x14, 7);

  return result;
}


bool Robotics::Servo::set_limit_T(double max)
{
  bool result = false;
  servo_buf[0] = DEGC_TO_RAW_T(max);
  result = command(0x18, 4);
  return result;
}


bool Robotics::Servo::get_limit(double& min, double& max)
{
  bool result = false;
  uint8_t dlc = 3;

  if (command_response(0x15, dlc))
  {
    if (dlc == 7)
    {
      min = RAW_ANG_TO_DEG(UINT16_BUILD(servo_buf[1], servo_buf[0]));
      max = RAW_ANG_TO_DEG(UINT16_BUILD(servo_buf[3], servo_buf[2]));
      result = true;
    }
  }
  return result;
}


bool Robotics::Servo::get_limit_V(double& min, double& max)
{
  bool result = false;
  uint8_t dlc = 3;

  if (command_response(0x17, dlc))
  {
    if (dlc == 7)
    {
      min = RAW_V_TO_VOLTS(UINT16_BUILD(servo_buf[1], servo_buf[0]));
      max = RAW_V_TO_VOLTS(UINT16_BUILD(servo_buf[3], servo_buf[2]));
      result = true;
    }
  }

  return result;
}


bool Robotics::Servo::get_limit_T(double& max)
{
  bool result = false;
  uint8_t dlc = 3;

  if (command_response(0x19, dlc))
  {
    if (dlc == 4)
    {
      max = RAW_T_TO_DEGC(servo_buf[0]);
      result = true;
    }
  }

  return result;
}


bool Robotics::Servo::set_limit_V(double min, double max)
{
  bool result = false;
  uint16_t pmin = VOLTS_TO_RAW_V(min);
  uint16_t pmax = VOLTS_TO_RAW_V(max);

  servo_buf[0] = UINT16_LOWER(pmin);
  servo_buf[1] = UINT16_UPPER(pmin);
  servo_buf[2] = UINT16_LOWER(pmax);
  servo_buf[3] = UINT16_UPPER(pmax);

  result = command(0x16, 7);

  return result;
}


// -----------------
// Full Servo Report
// -----------------

bool Robotics::Servo::report(Servo_Report& report)
{
  bool bResult = false;

  // clear report to baseline
  report.ID = id;
  report.IS_MOTOR = false;
  report.SERVO.A = INVALID_ANGLE;
  report.SERVO.A_min = INVALID_ANGLE;
  report.SERVO.A_max = INVALID_ANGLE;
  report.SERVO.OFFSET = INVALID_OFFSET;
  report.SERVO.IS_LOADED = false;
  report.V = INVALID_VOLTAGE;
  report.V_min = INVALID_VOLTAGE;
  report.V_max = INVALID_VOLTAGE;
  report.T = INVALID_TEMP;
  report.T_max = INVALID_TEMP;
  report.LED = false;
  report.LED_error = 0xFF;

  // fill the report
  if (is_attached())
  {
    bool bMotor;
    servo_speed_t speed;
    
    bResult = true;

    get_mode(bMotor, speed);
    if (bMotor)
    { // motor mode
      report.IS_MOTOR = true;
      report.MOTOR.SPEED = speed;
    }
    else
    { // servo mode
      report.IS_MOTOR = false;
      report.SERVO.A = angle();
      get_limit(report.SERVO.A_min, report.SERVO.A_max);
      report.SERVO.OFFSET = offset();
      get_load(report.SERVO.IS_LOADED);
    }
    report.V = V();
    get_limit_V(report.V_min, report.V_max);
    report.T = T();
    get_limit_T(report.T_max);
    report.LED = LED();
    report.LED_error = LED_error();
  }

  return bResult;
}


// ---------------
// Servo ID Change
// ---------------

// call unlock_id() before calling change_id()
// pass the result from unlock_id() as the code to change_id()
// prevents accidental changing of id
uint16_t Robotics::Servo::unlock_id()
{
  return prbs_accum;
}


// change_id must be preceded by a call to unlock_id
bool Robotics::Servo::change_id(servo_id_t new_id, uint16_t code)
{
  bool bresult = false;

  if (code == prbs_accum)
  { // this is only the case if no commands have occurred between unlock_id() and change_id()
    if (new_id >= ID_MIN && new_id <= ID_MAX)
    {
      servo_buf[0] = new_id;
      bresult = command(0x0D, 4);

      if (bresult)
        id = new_id;
    }
  }

  return bresult;
}


// ------------------------------------
// Servo command and response functions
// ------------------------------------

bool Robotics::Servo::command(uint8_t cmd, uint8_t dlc)
{
  bool bresult = false;

  char szbuf[128];

  if (pbus)
  {
    bresult = pbus->write(id, dlc, cmd, servo_buf);
  }

  prbs_cycle();
  return bresult;
}


bool Robotics::Servo::command_response(uint8_t cmd, uint8_t &dlc)
{
  bool bresult = false;

  if (pbus)
    bresult = pbus->write_read(id, dlc, cmd, servo_buf, SERVO_BUFLEN);    // SERVO_BUFLEN was 2. Don't know why.

  prbs_cycle();
  return bresult;
}


// ----------------------------
// Servo ID lock code utilities
// ----------------------------

uint16_t Robotics::Servo::prbs_accum = 0x5AFE;

void Robotics::Servo::prbs_cycle()
{
  uint16_t prbs;

  if (prbs_accum & 0x0001)
    prbs = prbs_accum ^ 0x63A3;
  else
    prbs = prbs_accum;
  prbs_accum = (prbs >> 1) + ((prbs & 0x0001) ? 0x8000 : 0x0000);
}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
