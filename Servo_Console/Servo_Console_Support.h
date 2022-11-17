/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Support.h
  Description:
    Functions and definitions used to give modules access to Servo Console I/O

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once

#define CHAR_BUF_LEN      32

void ServoConsole_report_header();
void ServoConsole_report(char const* szrep);
void ServoConsole_report(const __FlashStringHelper *szrep);
void ServoConsole_report(int value);
void ServoConsole_report(unsigned long value);
void ServoConsole_report(double value);
void ServoConsole_syntax_error();
void ServoConsole_report_failure();
void ServoConsole_report_success();
void ServoConsole_help();


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
