/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
  Free for usage without warranty, expressed or implied; attribution required

  Filename   : Servo_Console_Sets.h
  Description:
    Interface to functions for reading angle, storing it, and moving to stored
    angles for the set of all known servos that have been enumerated.

  Created    : 10/30/2021
  Modified   : 10/30/2021
  Author     : Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
#pragma once

void Initialize_Position_Sets();
void Save_Positions(MatchState& ms);
void Goto_Positions(MatchState& ms);
void Dump_Positions(MatchState& ms);
void Set_Positions(MatchState& ms);
void Dump_Positions_All();
void Get_Positions(MatchState& ms);
void Get_Positions();
void Goto_Home();


/*******************************************************************************
  Copyright © 2021 Kerry S. Martin, martin@wild-wood.net
*******************************************************************************/
