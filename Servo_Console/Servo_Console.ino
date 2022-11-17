/*******************************************************************************
  Copyright © 2022 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console.ino
  Description:
    Implements the console for controlling servos on a servo bus.

  Created    : 10/30/2021
  Modified   : 5/7/2022
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#error "Serial servo control requires the MEGA2560 board"
#endif

#include <Arduino.h>
#include <Regexp.h>

#include "Servo_Console_Servos.h"
#include "Servo_Console_Sets.h"
#include "Servo_Console_Grid.h"

//#define CONSOLE_ECHO    // comment-out to disable echo of commands
//#define CONSOLE_DEBUG   // comment-out to disable extra debug text

// input buffer lengths
#define CMD_BUFLEN       80
#define LINE_BUFLEN      (CMD_BUFLEN+2)

// Console variables
int SC_state;
#define SC_STATE_SHOW_PROMPT 0
#define SC_STATE_WAIT_CMD    1

// Buffer to store Serial input
char inCmdBuffer[CMD_BUFLEN];         // raw input, indexed by inCmdIndex
char szCmdLine[LINE_BUFLEN];          // null-terminated command line
size_t inCmdIndex;


// TODO:
//   MONITOR ANGLE mode (display ONCE or CONTinuosly angle for all servos)
//     see DUMP which does it once

// TODO: Flash strings F("") for Regexp.h library
//void foo(const __FlashStringHelper* szfString)
//{
//  char buffer[64];
//  strcpy_P(buffer, (char*)pgm_read_word(szfString));
//  Serial.print(buffer);
//}


/*******************************************************************************
  Servo Console Reporting Functions
*******************************************************************************/

void ServoConsole_report_header()
{
#ifdef CONSOLE_ECHO
  Serial.print(F(" => "));
#endif
}


void ServoConsole_report(const __FlashStringHelper *szrep)
{
  ServoConsole_report_header();
  Serial.println(szrep);
}


void ServoConsole_report(char const* szrep)
{
  ServoConsole_report_header();
  Serial.print(szrep);
}

void ServoConsole_report(int value)
{
  ServoConsole_report_header();
  Serial.println(value);
}


void ServoConsole_report(double value)
{
  ServoConsole_report_header();
  Serial.println(value);
}


void ServoConsole_report(unsigned long value)
{
  ServoConsole_report_header();
  Serial.println(value);
}


void ServoConsole_syntax_error()
{
  ServoConsole_report(F("syntax error"));
}


void ServoConsole_report_failure()
{
  ServoConsole_report(F("failed"));
}


void ServoConsole_report_success()
{
  ServoConsole_report(F("success"));
}


/*******************************************************************************
  Servo Console Implementation
*******************************************************************************/

void setup()
{
  // initialize the serial port and show the boot prompt
  Serial.begin(115200);
  Serial.print(F("\n\nArduino Servo Console\n"));
  SC_state = SC_STATE_SHOW_PROMPT;

  // initialize the input buffer
  inCmdIndex = 0;

  // initialize everything else
  Initialize_Servos();
  Initialize_Position_Sets();
  Initialize_Grid();
}


void ServoConsole_help()
{
  Serial.print(F(
                 "\n\nHelp:\nID #\t\t\t\tSet ID of target servo\n"
                 "ID?\t\t\t\tGet current ID\n"
                 "ID*\t\t\t\tQuery IDs of all attached servos\n"
                 "angle?\t\t\t\tGet servo angle\n"
                 "Move [pos] {[time]} {Wait} {*}\tProgram servo movement\n"
                 "START {*}\t\t\tStart movement\n"
                 "STOP {*}\t\t\tStop movement\n"
                 "Vin?\t\t\t\tGet servo supply voltage\n"
                 "Temp?\t\t\t\tGet servo temperature\n"
                 "LOAD {*}\t\t\tLoad the servo(s)\n"
                 "UNLOAD {*}\t\t\tUnload the servo(s)\n"
                 "LIMit angle [min] [max]\t\tSet angle limits\n"
                 "LIMit Temp [min] [max]\t\tSet max temp limit\n"
                 "LIMit Vin [min] [max]\t\tSet Vin limits\n"
                 "LIMit angle ?\t\t\tGet angle limits\n"
                 "LIMit Temp ?\t\t\tGet max temp limit\n"
                 "LIMit Vin ?\t\t\tGet Vin limits\n"
                 "MODE SERVO {*}\t\t\tConfigure as servo\n"
                 "MODE MOTOR [speed]\t\tConfigure as motor at given speed\n"
                 "MODE?\t\t\t\tGet the current mode (and speed)\n"
                 "OFFSet [adj] {WRite}\t\tSet the angle offset\n"
                 "OFFSet WRite {*}\t\tWrite the offset to servo NVM\n"
                 "OFFSet?\t\t\t\tGet the angle offset of the servo\n"
                 "LED ON|OFF {*}\t\t\tTurn the LED on or off\n"
                 "LED?\t\t\t\tGet the current state of the LED\n"
                 "LED ERRor [0-7] {*}\t\tSet the LED error mode\n"
                 "LED ERRor ?\t\t\tGet the LED error mode\n"
                 "CHANGE ID [curid] TO [newid]\tChange the ID of the servo!\n"
                 "SAVE #\t\t\t\tStore servo angles as position set #\n"
                 "GOto #\t\t\t\tGoto servo position set #\n"
                 "HOME\t\t\t\tGoto servo position set 0\n"
                 "Dump {*}\t\t\tDump all of the known sets\n"
                 "Dump #\t\t\t\tDump set #\n"
                 "Dump Servos\t\t\tDump the current values of all servos\n"
                 "SET # = a1 a2 ... an\t\tAssign the values to set #\n"
                 "GET #\t\t\t\tGet SET command for set #\n"
                 "GET {*}\t\t\tGet SET commands for all sets\n"
                 "REPORT\t\t\t\tReport info on all servos\n"

                 "  [] => parameter value\n"
                 "  {} => optional\n"
                 "  lower-case denotes abbreviated characters\n"
               ));
}


bool FormCmdLine()
{
  bool bResult = false;

  while (Serial.available() > 0)
  {
    char inChar = char(Serial.read()); // get next character

    switch (inChar)
    {
      case '\b':  case '\x7F':  // backspace or del deletes the previous character
        if (inCmdIndex > 0)
          --inCmdIndex;
        break;

      case '\r':  // ignore carriage return
        break;

      case '\n':  // line-feed ends the line
        // drop trailing spaces
        while (inCmdIndex > 0)
        {
          if (inCmdBuffer[inCmdIndex-1] != ' ')
            break;
          else
            --inCmdIndex;
        }

        if (inCmdIndex > 0)
        {
          strncpy(szCmdLine, inCmdBuffer, inCmdIndex);
          szCmdLine[inCmdIndex] = '\0';
          bResult = true;
          inCmdIndex = 0; // start fresh
        }
        break;

      case '\t':  // change a tab character to a space
        inChar = ' ';
      // NO BREAK - PASS THROUGH

      default:
        // add every other non-control character to the buffer
        if (inChar >= '\x20' && inChar <= '\x7E')
        {
          // convert lowercase to uppercase
          if (inChar >= 'a' && inChar <= 'z')
            inChar = inChar - 'a' + 'A';

          // drop leading spaces
          if (inChar == ' ' && inCmdIndex == 0)
            break;

          // store it in the buffer
          inCmdBuffer[inCmdIndex++] = inChar;

          // do a buffer overflow check
          if (inCmdIndex >= CMD_BUFLEN)
            inCmdIndex = 0; // buffer was full, start fresh
        }
        break;
    }

    if (bResult)
      break;
  }

  return bResult;
}


void loop()
{
  MatchState ms;

  switch (SC_state)
  {
  default: case SC_STATE_SHOW_PROMPT:
      Serial.print(F("> "));
      SC_state = SC_STATE_WAIT_CMD;
      break;

    case SC_STATE_WAIT_CMD:
      if (FormCmdLine())
      {
        ms.Target(szCmdLine);
#ifdef CONSOLE_DEBUG
        Serial.print(F("Received line: \""));
        Serial.print(szCmdLine);
        Serial.print(F("\"\n"));
#endif

#ifdef CONSOLE_ECHO
        Serial.print(szCmdLine);
#endif

        if (ms.Match("^HE?L?P?$"))
          ServoConsole_help();
        else if (ms.Match("^DIST%?$"))
          Distance_Query();
        else if (ms.Match("^LIMI?T?%s+TE?M?P?%s*%?$"))
          Limit_Temp_Query();
        else if (ms.Match("^LIMI?T?%s+VI?N?%s*%?$"))
          Limit_Vin_Query();
        else if (ms.Match("^LIMI?T?%s*A?N?G?L?E?%s*%?$"))
          Limit_Angle_Query();
        else if (ms.Match("^LIMI?T?%s+TE?M?P?%s+(%d+)%s*(%*?)$"))
          Limit_Temp(ms);
        else if (ms.Match("^LIMI?T?%s+VI?N?%s+([%d.]+)%s+([%d.]+)%s*(%*?)$"))
          Limit_Vin(ms);
        else if (ms.Match("^LIMI?T?%s*A?N?G?L?E?%s+([%d.]+)%s+([%d.]+)%s*(%*?)$"))
          Limit_Angle(ms);
        else if (ms.Match("^A?N?G?L?E?%s*%?$"))
          Angle_Query();
        else if (ms.Match("^VI?N?%s*%?$"))
          Vin_Query();
        else if (ms.Match("^TE?M?P?%s*%?$"))
          Temp_Query();
        else if (ms.Match("^CHANGE%s+ID%s+(%d+)%s+TO%s+(%d+)$"))
          Change_ID(ms);
        else if (ms.Match("^MODE%s+SERVO$"))
          Mode_Servo();
        else if (ms.Match("^MODE%s+SERVO%s+%*$"))
          Mode_Servo_Broadcast();
        else if (ms.Match("^MODE%s+MOTOR%s+(%-?%d+)$"))
          Mode_Motor(ms);
        else if (ms.Match("^MODE%s*%?$"))
          Mode_Query();
        else if (ms.Match("^OFFSE?T?%s*%?$"))
          Offset_Query();
        else if (ms.Match("^OFFSE?T?%s+(%-?%d+)$"))
          Angle_Offset(ms);
        else if (ms.Match("^OFFSE?T?%s+(%-?%d+)%s+WRI?T?E?$"))
          Angle_Offset_Write(ms);
        else if (ms.Match("^OFFSE?T?%s+WRI?T?E?$"))
          Angle_Offset_Write_Current();
        else if (ms.Match("^OFFSE?T?%s+WRI?T?E?%s+%*$"))
          Angle_Offset_Write_Current_Broadcast();
        else if (ms.Match("^LED%s*%?$"))
          LED_Query();
        else if (ms.Match("^LED%s+ON%s*(%*?)$"))
          LED_On(ms);
        else if (ms.Match("^LED%s+OFF%s*(%*?)$"))
          LED_Off(ms);
        else if (ms.Match("^LED%s+ERRO?R?%s*%?$"))
          LED_Error_Query();
        else if (ms.Match("^LED%s+ERRO?R?%s+([0-7])%s*(%*?)$"))
          LED_Error(ms);
        else if (ms.Match("^ID%s+(%d+)$"))
          Set_Active_ID(ms);
        else if (ms.Match("^ID%s*%?$"))
          ID_Query();
        else if (ms.Match("^ID%s*%*$"))
          Enumerate_IDs();
        else if (ms.Match("^START%s+%*$"))
          Move_Start_Broadcast();
        else if (ms.Match("^START$"))
          Move_Start();
        else if (ms.Match("^STOP%s+%*$"))
          Move_Stop_Broadcast();
        else if (ms.Match("^STOP$"))
          Move_Stop();
        else if (ms.Match("^LOAD%s+%*$"))
          Load_Servos_Broadcast();
        else if (ms.Match("^LOAD$"))
          Load_Servo();
        else if (ms.Match("^UNLOAD%s+%*$"))
          Unload_Servos_Broadcast();
        else if (ms.Match("^UNLOAD$"))
          Unload_Servo();
        else if (ms.Match("^MO?V?E?%s+(%d+)(.*)$"))
          Move_Servos(ms);
        else if (ms.Match("^SAVE%s+(%d+)$"))
          Save_Positions(ms);
        else if (ms.Match("^HOME$"))
          Goto_Home();
        else if (ms.Match("^GO?T?O?%s+(%d+)$"))
          Goto_Positions(ms);
        else if (ms.Match("^DU?M?P?%s+([%w]+)$"))
          Dump_Positions(ms);
        else if (ms.Match("^DU?M?P?%s*%*?$"))
          Dump_Positions_All();
        else if (ms.Match("^SET%s+(%d+)%s*=%s*([%d%s]+)$"))
          Set_Positions(ms);
        else if (ms.Match("^GET%s+(%d+)$"))
          Get_Positions(ms);
        else if (ms.Match("^GET%s*%*?$"))
          Get_Positions();
        else if (ms.Match("^REPORT$"))
          Servo_Report();
        else if (ms.Match("^REPORT%s+(%d+)$"))
          Servo_Report(ms);
        else
          ServoConsole_syntax_error();

        // end the line
#ifdef CONSOLE_ECHO
        Serial.println(F(""));
#endif
        // return to the prompt
        SC_state = SC_STATE_SHOW_PROMPT;
      }

      break;
  }
}


/*******************************************************************************
  Copyright © 2022 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
