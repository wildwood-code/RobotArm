/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Servos.cpp
  Description:
    Interface to the functions for controlling the servos via the console.
    
  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once

void Initialize_Servos();
void Angle_Query();
void Vin_Query();
void Temp_Query();
void Offset_Query();
void ID_Query();
void Limit_Temp(MatchState& ms);
void Limit_Temp_Query();
void Limit_Vin_Query();
void Limit_Angle_Query();
void Limit_Vin(MatchState& ms);
void Limit_Angle(MatchState& ms);
void Change_ID(MatchState& ms);
void Mode_Servo();
void Mode_Servo_Broadcast();
void Mode_Motor(MatchState& ms);
void Mode_Query();
void Angle_Offset(MatchState& ms);
void Angle_Offset_Write(MatchState& ms);
void Angle_Offset_Write_Current();
void Angle_Offset_Write_Current_Broadcast();
void LED_Query();
void LED_On(MatchState& ms);
void LED_Off(MatchState& ms);
void LED_Error_Query();
void LED_Error(MatchState& ms);
void Set_Active_ID(MatchState& ms);
void Enumerate_IDs();
void Move_Start_Broadcast();
void Move_Start();
void Move_Stop_Broadcast();
void Move_Stop();
void Load_Servos_Broadcast();
void Load_Servo();
void Unload_Servos_Broadcast();
void Unload_Servo();
void Move_Servos(MatchState& ms);
void Servo_Report();
void Servo_Report(MatchState& ms);


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
