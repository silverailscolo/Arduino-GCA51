/**************************************************************************
    LocoIno - Configurable Arduino Loconet Module - parts used in GCA51
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
    This software emulates the functionality of a GCA50 board from Peter
    Giling (Giling Computer Applications). This is a Loconet Interface
    with 16 I/O that can be individually configured as Input (block sensors)
    or Outputs (switches, lights,...).
    Configuration is done through SV Loconet protocol and can be configured
    from Rocrail (Programming->GCA->GCA50).
 ------------------------------------------------------------------------
 CREDITS:
 * Based on MRRwA Loconet libraries for Arduino - http://mrrwa.org/ and
   the Loconet Monitor example.
 * Inspired in GCA50 board from Peter Giling - http://www.phgiling.net/
 * Idea also inspired in LocoShield from SPCoast - http://www.scuba.net/
 * Thanks also to Rocrail group - http://www.rocrail.org
 ------------------------------------------------------------------------
 LAST CHANGES:
 1/9/2019 - Inform state of all inputs at power on, depends on the define #INFORMATPOWERON
          - Bug fixed on input numbers, they are stored in value1 and value2 different than outputs
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
*               The pointer is actualy the name of this function
**********************************************************************************************************************/
void notifyPower( uint8_t State )
{
  int n;
  int currentState;

  #ifdef DEBUG
  Serial.print("POWER: ");
  Serial.println( State ? "ON" : "OFF" );
  #endif

  #ifdef INFORMATPOWERON
  if (State)
  {
    // Check inputs to inform
    for (n=0; n<16; n++)
    {
      if (!bitRead(svtable.svt.pincfg[n].cnfg,7) && software_address[n]>1) // Setup as an Input greater than 1
      {
        currentState=digitalRead(pinMap[n]);

        #ifdef DEBUG
        Serial.print("INPUT ");Serial.print(n);
        Serial.print(" IN PIN "); Serial.print(pinMap[n]);
        Serial.print(" INFORMED AT POWER: "); Serial.print(software_address[n]); Serial.print(" = "); Serial.println(!currentState);
        #endif
        bitWrite(svtable.svt.pincfg[n].value2,4,!currentState);
        LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);
        //Update state to detect flank (use bit in value2 of SV)
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
*               The pointer is actualy the name of this function
**********************************************************************************************************************/
void notifySensor( uint16_t Address, uint8_t State )
{
  #ifdef DEBUG
  Serial.print("Sensor: ");
  Serial.print(Address, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
  #endif
}

/*********************************************************************************************************************
* Function    : void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
* Description : This call-back function is called from LocoNet.processSwitchSensorMessage for all Sensor messages
*               In the LocoNet.processSwitchSensorMessage is a pointer to this function
*               The pointer is actualy the name of this function
**********************************************************************************************************************/
void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

/*********************************************************************************************************************
* Function    : void SwitchPulseContact_OFF (int contact)
* Description :
**********************************************************************************************************************/
// TODO
