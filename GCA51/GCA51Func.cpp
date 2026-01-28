/**************************************************************************
    LocoIno - Configurable Arduino LocoNet Module - some parts used in GCA51
    Copyright (C) 2014-2024 Daniel Guisado Serra

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ------------------------------------------------------------------------
 AUTHOR : Dani Guisado - http://www.clubncaldes.com - dguisado@gmail.com
 ------------------------------------------------------------------------
 DESCRIPTION:
    GCA51 Library of _identical_ common functions with GCA50a.
*************************************************************************/

#ifndef LOCOGCA50_H_
  #define LOCOGCA50_H_
#endif

#include <LocoNet.h>
#include <EEPROM.h>


/*************************************************************************/
/*          LOCONET FUNCTIONS                                            */
/*************************************************************************/

/*********************************************************************************************************************
* Function    : void notifyPower( uint8_t State )
* Description : This call-back function is called from LocoNet.processSwitchSensorMessage for all Sensor messages
*               In the LocoNet.processSwitchSensorMessage is a pointer to this function
*               The pointer is actually the name of this function
**********************************************************************************************************************/
void notifyPower( [[maybe_unused]] uint8_t State )
{
  #ifdef DEBUG
  Serial.print("POWER: ");
  Serial.println( State ? "ON" : "OFF" );
  #endif

  #ifdef INFORMATPOWERON
  if (State)
  {
    // Check inputs to inform
    for (int n=0; n<16; n++)
    {
      if (!bitRead(svtable.svt.pincfg[n].cnfg,7) && software_address[n]>1) // Setup as an Input greater than 1
      {
        int currentState=digitalRead(pinMap[n]);

        #ifdef DEBUG
        Serial.print("INPUT ");Serial.print(n);
        Serial.print(" IN PIN "); Serial.print(pinMap[n]);
        Serial.print(" INFORMED AT POWER: "); Serial.print(software_address[n]); Serial.print(" = "); Serial.println(!currentState);
        #endif
        bitWrite(svtable.svt.pincfg[n].value2,4,!currentState);
        LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);
        // Update stored state to detect flank (use bit in value2 of SV)
        bitWrite(svtable.svt.pincfg[n].value2,4,currentState);
      }
    }
  }
  #endif
}

/*********************************************************************************************************************
* Function    : void notifySensor( uint16_t Address, uint8_t State )
* Description : This call-back function is called from LocoNet.processSwitchSensorMessage for all Sensor messages
*               In the LocoNet.processSwitchSensorMessage is a pointer to this function
*               The pointer is actually the name of this function
**********************************************************************************************************************/
void notifySensor( [[maybe_unused]] uint16_t Address, [[maybe_unused]] uint8_t State )
{
  #ifdef DEBUG
  Serial.print(F("Sensor: "));
  Serial.print(Address, DEC);
  Serial.print(F(" - "));
  Serial.println( State ? "Active" : "Inactive" );
  #endif
}

/*********************************************************************************************************************
* Function    : void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
* Description : This call-back function is called from LocoNet.processSwitchSensorMessage for all Sensor messages
*               In the LocoNet.processSwitchSensorMessage is a pointer to this function
*               The pointer is actually the name of this function
**********************************************************************************************************************/
void notifySwitchReport( [[maybe_unused]] uint16_t Address, [[maybe_unused]] uint8_t Output, [[maybe_unused]] uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print(F("Switch Report: "));
  Serial.print(Address, DEC);
  Serial.print(F(':'));
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(F(" - "));
  Serial.println(Output ? "On" : "Off");
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch State messages
void notifySwitchState( [[maybe_unused]] uint16_t Address, [[maybe_unused]] uint8_t Output, [[maybe_unused]] uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print(F("Switch State: "));
  Serial.print(Address, DEC);
  Serial.print(F(':'));
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(F(" - "));
  Serial.println(Output ? "On" : "Off");
  #endif
}

/*********************************************************************************************************************
* Function    : void SwitchPulseContact_OFF (int contact)
* Description :
**********************************************************************************************************************/
// TODO
