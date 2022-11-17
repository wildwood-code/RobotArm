/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo.h
  Class      : Servo
  Description:
    The Servo class encapsulates the movement and control functions of the
    HiWonder/LewanSoul bus servos (LX-16A, LX-15D, LX-224, LX-225, LX-824,
    and LX-1501).

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once

#include "ServoBus.h"

namespace Robotics
{

typedef uint8_t servo_id_t;             // Servo identifier
typedef double servo_angle_t;           // Servo angle in degrees
typedef uint16_t servo_angle_raw_t;     // Servo angle in raw binary
typedef double servo_voltage_t;         // Servo supply voltage in V
typedef uint16_t servo_voltage_raw_t;   // Servo supply voltage in raw binary
typedef double servo_temp_t;            // Servo temperature in degC
typedef uint8_t servo_temp_raw_t;       // Servo temperature in raw binary
typedef int16_t servo_speed_t;          // Servo speed in rpm (+/-)
typedef int16_t servo_offset_raw_t;     // Servo angle offset in raw binary (+/-)

#define SERVO_BUFLEN 16                 // Buffer length for servo communications

// ID definitions
#define ID_MIN       (Robotics::servo_id_t(0x00))      // first valid servo ID
#define ID_MAX       (Robotics::servo_id_t(0xFD))      // last valid servo ID (excluding broadcast and reserved)
#define ID_BROADCAST (Robotics::servo_id_t(0xFE))      // ID for broadcast to all servos on bus
#define ID_RESERVED  (Robotics::servo_id_t(0xFF))      // reserved ID, use for non-initialized Servo

// invalid value indicators
#define INVALID_ANGLE    (Robotics::servo_angle_t(-999.9))
#define INVALID_VOLTAGE  (Robotics::servo_voltage_t(-999.9))
#define INVALID_TEMP     (Robotics::servo_temp_t(-999.9))
#define INVALID_OFFSET   (servo_offset_raw_t(-2048))
#define INVALID_SPEED    (servo_speed_t(-9999))
#define INVALID_LED_FAULT (uint8_t(255))


struct Servo_Report
{
  servo_id_t ID;
  bool IS_MOTOR;
  union
  {
    struct
    {
      servo_angle_t A, A_min, A_max;
      servo_offset_raw_t OFFSET;
      bool IS_LOADED;
    } SERVO;
    struct
    {
      servo_speed_t SPEED;
    } MOTOR;
  };
  servo_voltage_t V, V_min, V_max;
  servo_temp_t T, T_max;

  bool LED;
  uint8_t LED_error;
};


class Servo
{
  public:
    // Construction/destruction
    Servo(servo_id_t id = ID_RESERVED, ServoBus* pbus = NULL);
    virtual ~Servo();
    Servo& operator = (Servo const&) = delete;

    // Reassign ID
    void operator = (servo_id_t id) {
      Servo::id = id;
    }
    void clear_id() {
      Servo::id = ID_RESERVED;
    }
    operator servo_id_t () const {
      return Servo::id;
    }

    // Connection to the ServoBus
    bool attach(ServoBus& bus);       // attach servo to the bus
    bool detach();                    // detach servo from the bus
    bool is_attached();               // does servo respond on the bus?

    // Movement
    servo_angle_t move(servo_angle_t deg, uint16_t msec);       // set position in degrees at time in ms
    servo_angle_t move_wait(servo_angle_t deg, uint16_t msec);  // set delayed move
    servo_angle_t angle();                                      // read current position in degrees
    bool start();                                               // start delayed move
    bool stop();                                                // stop delayed move
    servo_angle_t tol() const;                                  // tolerance on angle for move complete
    servo_angle_t tol(servo_angle_t deg_tol);                   // set tolerance on angle for move complete
    bool is_move_complete();                                    // is servo at target position?
    bool load(bool on = true);                                  // loads the servo (holds position)
    bool unload(bool off = true);                               // unloads the servo (position is free to be moved)
    bool get_load(bool& load);                                  // reads load(true)/unload(false) state

    // Offset adjustment
    bool offset(servo_offset_raw_t offs, bool write = false);   // configure the offset and optionally write to NVM
    bool offset_write();                                        // write the configured offset to NVM
    servo_offset_raw_t offset();                                // get the configured offset

    // Servo mode
    bool set_mode(bool motor, servo_speed_t speed = 0);             // set servo to servo or motor mode, and set speed in motor mode
    bool speed(servo_speed_t speed);                            // set the speed (and sets it to motor mode)
    servo_speed_t speed();                                      // get the speed
    bool is_servo();                                            // true if it is configured as a servo
    bool is_motor();                                            // true if it is configured as a motor
    bool get_mode(bool& motor, servo_speed_t& speed);         // read the motor state and speed

    // limits for angle, voltage, and temperature
    bool set_limit(servo_angle_t min, servo_angle_t max);               // set the min/max angle limits
    bool set_limit_T(servo_temp_t max);                                 // set the max temperature limit
    bool set_limit_V(servo_voltage_t min, servo_voltage_t max);         // set the min/max voltage limits
    bool get_limit(servo_angle_t& min, servo_angle_t& max);         // get the min/max angle limits
    bool get_limit_T(servo_temp_t& max);                            // get the max temperature limit
    bool get_limit_V(servo_voltage_t& min, servo_voltage_t& max);   // get the min/max voltage limits

    // Voltage and temperature
    servo_voltage_t V();                                      // read voltage in Volts
    servo_temp_t T();                                         // read temperature in degC

    // Light Emitting Diode (LED)
    bool LED();                                               // read LED state
    bool LED(bool led);                                       // set LED state  (0=off, 1=on)
    uint8_t LED_error();                                      // read LED error config
    bool LED_error(uint8_t fault);                            // set LED error config

    // Full Servo Report
    bool report(Servo_Report& report);                        // Get report of all Servo info

    // Changing the Servo ID (this is a two step process)
    uint16_t unlock_id();                                     // get the code to unlock the servo id (step 1)
    bool change_id(servo_id_t new_id, uint16_t code);         // change the ID of the servo (step 2)

  private:
    servo_id_t   id;                                          // ID of the servo: 0-253 is normal, 254 is broadcast, 255 is reserved
    uint16_t  tgt_angle;                                      // holds the target angle
    uint16_t  tol_angle;                                      // holds the tolerance of the target angle
    ServoBus  *pbus;                                          // pointer to the ServoBus to which servo is attached
    uint8_t   servo_buf[SERVO_BUFLEN];                        // buffer for transmitting and receiving servo data

    // Commands and responses
    bool command(uint8_t cmd, uint8_t dlc);                   // issue a command
    bool command_response(uint8_t cmd, uint8_t &dlc);         // issue a command and wait for the response

    // PRBS used for unlocking the Servo ID
    static uint16_t  prbs_accum;                              // pseudo-random number generator accumulator
    static void prbs_cycle();                                 // cycle to the next pseudo-random number
};

}


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
