 /**************************************************************************
    LocoGCA51 - Configurable Arduino LocoNet Module
    Copyright (C) 2016 Gerard Remmerswaal

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
 DESCRIPTION:
    This software uses two RFID readers to read RFID tags.
    The data from the tags are sent to Rocrail with LocoNet communication.
    On the GCA51 board are 8 extra inputs available for inputs/outputs.
    Configuration is done through SV LocoNet protocol and can be configured
    from Rocrail (Programming->GCA->GCA50).
    The "heart" of the GCA51 is an Arduino Nano board.
 ------------------------------------------------------------------------
 PIN ASSIGNMENT:
   0,1 -> Serial, used to debug and LocoNet Monitor (uncomment DEBUG)
   2,3,4,5,6 -> Configurable I/O from 1 to 5
   7 -> LocoNet TX (connected to GCA185 shield)
   8 -> LocoNet RX (connected to GCA185 shield)
   9,10,11,12,13 -> Configurable I/O from 6 to 10
   A0,A1,A2,A3,A4,A5-> Configurable I/O from 11 to 16
 ------------------------------------------------------------------------
 CREDITS: 
 * Based on MRRwA LocoNet libraries for Arduino - http://mrrwa.org/ and 
   the LocoNet Monitor example.
 * Inspired in GCA50 board from Peter Giling - http://www.phgiling.net/
 * Idea also inspired in LocoShield from SPCoast - http://www.scuba.net/
 * Thanks also to Rocrail group - http://www.rocrail.org
 * Thanks to the LocoNet part of software from Dani Guisado/ClubNCaldes
*************************************************************************/

#include <LocoNet.h>
#include <SPI.h>
#include <EEPROM.h>
#include "LocoGCA51.h"
#include "rfid2ln.h"
#include <MFRC522.h>
#include <Arduino.h>

#define DEBUG                                  // Uncomment this line to debug through the serial monitor
#define LN_TX_PIN       7                      // Arduino Pin used as LocoNet Tx; Rx Pin is always the ICP Pin 
#define RST_PIN         6                      // Arduino Pin used as ResetPowerDownPin
#define SDA_1_PIN      10                      // The SDA_1 pin provides the Arduino with tag data from Reader 1
#define SDA_2_PIN       9                      // The SDA_2 pin provides the Arduino with tag data from Reader 2
#define LocoLED         4                      // LocoLED lights up when there is LocoNet communication
#define LocoLED_wait  200                      // LocoLED is always on for 200 msec
#define PulseTime     300                      // PulseTime for all the Pulse Outputs in msec.
#define WaitTime      500                      // Wait Time for all the Inputs in msec.
#define FlashTime     250                      // Frequency of flasher

unsigned long ResetUid_1;                      // stores the time moment when the old UID from Reader 1 was stored.
unsigned long ResetUid_2;                      // stores the time moment when the old UID from Reader 2 was stored.

LocoNetSystemVariableClass sv;
lnMsg       *LnPacket;                         // pointer naar lnMsg
lnMsg       SendPacketSensor ;                 // SendPacketSensor is now a uint8_t data[16]  array.
boolean bSerialOk=true;   

unsigned char oldUid [2][7];                   // Two Readers with 7 bytes of information from each tag

// Arduino pin assignment to each of the 16 outputs
// Actualy the Arduino has only 14 digital IO ( 0 till 13). You can however use the analog inputs 0 - 5 also as digital IO.
// To change Analog input 0 to a digital IO you should use pin number 14 ==>  pinMode (14, OUTPUT);
// The second Analog input will give you pinMode 15 etc. The GCA185 board uses in this way all the availible IO pins.
// To connect two RFID pcb's on the GCA185 you need the Arduino pin numbers 2, 10, 11, 12 and 13, so you cannot use these pins for another purpose.
// The LocoNet pins, needed for the communication, are the Arduino pin numbers 7 and 8

byte     pinMap[8] = {14,15,16,17,18,19,2,3};                         // The analog inputs are used as digital I/O and that is why the names of these ports are changed.  So, A0 == D14,  A1 == D15,  A2 == D16, ........A7 == D21
                                                                      // 2 and 3 are D2 and D3. These to IO use INT0 and INT1                                              
uint16_t Software_address[16];                                        // composite software address of all the hardware ports
volatile byte IO_status[8] = {0,0,0,0,0,0,0,0};                       // remember the last IO status.  A '1' means the port is active.  "0" means the port is not active

/*******
* 3 bytes defining a pin behavior and software adress of the hardware ports  ( http://wiki.rocrail.net/doku.php?id=loconet-io-en )
*******/

struct PIN_CFG
{
  uint8_t cnfg;
  uint8_t value1;
  uint8_t value2;
}; 

//Memory map exchanged with SV read and write commands ( http://wiki.rocrail.net/doku.php?id=lnsv-en )
struct SV_TABLE
{
  uint8_t vrsion;
  uint8_t addr_low;
  uint8_t addr_high;
  PIN_CFG pincfg[16];                    // pincfg[16] has all the hardware / software settings of the 16 I/O ports.  cnfg, value1 and value2
};

union SV_DATA                            //Union to access the data with the struct or by index
 {
   SV_TABLE svt;
   uint8_t data[51];
 };
SV_DATA svtable;                        // Union declaration svtable

// ********************************** RFID ***************************************

MFRC522 mfrc522_1(SDA_1_PIN, RST_PIN);           // Declaration of the first MFRC522 instance.
MFRC522 mfrc522_2(SDA_2_PIN, RST_PIN);           // Declaration of the second MFRC522 instance. 

MFRC522::MIFARE_Key key;

/*********************************************************************************  
 *  void LocoNet_communication ()
 *  Turn LocoLED on  when LocoNet communication starts
 *  Turn LocoLED off when the waiting time has elapsed
 *********************************************************************************/
 void LocoNet_communication (byte on_off)
 {
    static unsigned long LED_on_off; 
    
    if (on_off == 1)                         // LocoLED is off
    {
      digitalWrite (LocoLED, HIGH);         // LocoNet communication is started, switch on LocoLED
      LED_on_off = millis();                // remember start time
    }

    if ((LED_on_off + LocoLED_wait) < millis())     // if the wait time is expired, 
    {
      LED_on_off = 0;                               
      digitalWrite (LocoLED, LOW);                  // switch off LocoLED
    }
 }

/*********************************************************************************  
 *   void CalculateAddres ()
 *  In this function the software adresses are calculated and stored in the global variable Software_address[16]
 *********************************************************************************/
void CalculateAddres ()
{
  int n;
  byte odd_even;

  
  for (n = 0; n < 16; n++)             
  {                                                                    
    if (svtable.svt.pincfg[n].cnfg == 27)                                                         // declared as input
      {
        if   (bitRead (svtable.svt.pincfg[n].value2, 5)) odd_even = 2;                            // bitread for bit 5 on SV5, SV8, SV11, SV14 etc.
        else odd_even = 1; 
             
        Software_address[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1 << 1 ) + odd_even);  // Caluculated software address of the port. For Port 1 .value1 == SV4 and .value2 == SV5
        //           (SV5 & 0x0F) << 8 == high byte +  SV4 << 1 == low byte + odd_even == software-address of the hardware-port
        Serial.print ("berekend adres input  : "); Serial.println(Software_address[n], DEC);
      }
    if ((svtable.svt.pincfg[n].cnfg == 128) || ((svtable.svt.pincfg[n].cnfg == 129)))              // declared as output
      {
        Software_address[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 7 ) + (svtable.svt.pincfg[n].value1)+ 1);  // Caluculated software address of the port. For Port 1 .value1 == SV4 and .value2 == SV5
        Serial.print ("berekend adres output  : "); Serial.println(Software_address[n], DEC);
      }
    
  }
}

//***************************************************************************************************************************
// * Function : void InitialiseInterrupt()
// * This function is the last function called in de setup() routine when all the IO's have their function, input or output 
// * Only the inputs will get assigned an interrupt
// **************************************************************************************************************************
void InitialiseInterrupt()
{
  int n;
  cli();                    // switch interrupts off while messing with their settings  
  PCICR =0x02;              // PCIE1 interrupt is enabled for pin group A0 .. A7
  
  for (n=8;n<14;n++)        // only inputs should give an interupt.  Select A0..A5
  {
    if (bitRead(svtable.svt.pincfg[n].cnfg,7) == 0)       // IO port is an input
        bitWrite (PCMSK1, n-8, 1);                        // interrupts on pin A0 .. A5
  }
  
  if (bitRead(svtable.svt.pincfg[14].cnfg,7) == 0) bitWrite (EIMSK, 0,1);    // if D2 is an input ==> enable INT0
  if (bitRead(svtable.svt.pincfg[15].cnfg,7) == 0) bitWrite (EIMSK, 1,1);    // if D3 is an input ==> enable INT1
 
  EICRA  = 0b00001010;      // D2 INT0 on falling Edge.    D3 INT1 on falling Edge.
  sei();                    // turn interrupts back on
}

ISR(INT0_vect)              // ISR(INT0_vect) only reacts on a descending edge at the input
{ IO_status[6] = 1; }       //Serial.println("D2");}

ISR(INT1_vect)              // ISR(INT1_vect) only reacts on a descending edge at the input
{ IO_status[7] = 1; }       //Serial.println("D3");}

ISR(PCINT1_vect)        // Interrupt service routine. Every single PCINT8..14 (=ADC0..5) change will generate an interrupt: but this will always be the same interrupt routine
{     
  if (digitalRead(A0)==0)  IO_status[0] = 1;    //Serial.println("A0");
  if (digitalRead(A1)==0)  IO_status[1] = 1;    //Serial.println("A1");
  if (digitalRead(A2)==0)  IO_status[2] = 1;    //Serial.println("A2");
  if (digitalRead(A3)==0)  IO_status[3] = 1;    //Serial.println("A3");
  if (digitalRead(A4)==0)  IO_status[4] = 1;    //Serial.println("A4");
  if (digitalRead(A5)==0)  IO_status[5] = 1;    //Serial.println("A5");
}
  
void setup()
{
  int n;

  pinMode (LocoLED, OUTPUT);                    // LocoLED lights up when there is LocoNet communication
    
  start_setup();                                // Start values of the board in LocoGCA51.cpp,  read EEPROM values.        Function start_setup is in LocoDani.cpp       

  for (n = 0; n < 51; n++)                      // Load config from EEPROM
    svtable.data[n] = EEPROM.read(n);           // Read the values of SV0 till SV51. The values in EEPROM were allright or made standard in start_setup()

  CalculateAddres ();                           // In this function the software adresses are calculated and stored in the global variable Software_address[16]
  
  for (n=8;n<16;n++)                            // Configure I/O pins and give the output a start value. The first 8 IO ports are already set and are not available to users, except the adresses of port 1 and 2 (RFID sensors)
                                                // The actual hardware pinnumbers are declared in the global variabele pinMap[]
  {
    if (bitRead(svtable.svt.pincfg[n].cnfg,7))                                          // if bit 7 of the cnfg byte is '1', the pin is an output
    {                                                                                   // if bit 7 of the cnfg byte is '0', the pin is an input
      pinMode(pinMap[n-8],OUTPUT);
      if (bitRead(svtable.svt.pincfg[n].cnfg,0))  digitalWrite(pinMap[n-8], HIGH);      // if bit 0 van cnfg == 1 then the output is high at startup
      else  digitalWrite(pinMap[n-8], LOW);                                             // else the output is low at startup
    }
    else
    { 
       pinMode(pinMap[n-8],INPUT_PULLUP);
       bitWrite(svtable.svt.pincfg[n].value2,4,1);    // block detector, geen puls contact   
    }
  InitialiseInterrupt();                              // give only the inputs an interrupt
}


// ********************************** RFID ***************************************

    SPI.begin();                                                                                  // Init SPI bus
    
    mfrc522_1.PCD_Init();                                                                         // Init 1st MFRC522 card
    mfrc522_2.PCD_Init();                                                                         // Init 2nd MFRC522 card

// **************************** External Interrupts *****************************    


}   // end setup()



void loop()                   //*************** MAIN LOOP () *************************
{ 
  static int n, time_msec ;
  byte temp_IO;                                          
  static unsigned long IO_timing[8];                   // array[8] with Puls- or Debounce timing for each IO-port
  static unsigned long CurrentTime;                    // time of this moment
  static byte remember_input[8];                       // remembers wich input was active.  After "waittime" the program will reset this input

  LocoNet_communication (0);                           // switch off LocoLED when the wait time is expired
                                                                                         
  LnPacket = LocoNet.receive() ;                       // Check for any received LocoNet packets
  if( LnPacket )
  {
    #ifdef DEBUG
        print_out_LnPacket ();                         // function in LocoGCA51.cpp to print out LnPacket
    #endif  
    
    if(!LocoNet.processSwitchSensorMessage(LnPacket))  // If this packet was not a Switch or Sensor Message checks por PEER packet
    {      
      processPeerPacket();
    }
  }
  

            //******** INPUT *********                
  for (n=8; n<16; n++)                                                                                    // Check I/O ports 8 till 15.   I/O ports 0 till 8 are used for the communication with the RFID equipment
    {          
       if ((IO_timing[n-8] == 0) && (bitRead(svtable.svt.pincfg[n].cnfg,7)) == 0)                         // there is no WaitTime active && the port is an input port
          { 
            if (IO_status[n-8] == 1)                                                                      // IO_status contains the last known value.  At the ISR's this array is filled with new input information
              {
                
                IO_timing[n-8] = millis();                                                                // input is active, then start timer    
                LocoNet_communication (1);                                                                // LocoNet communication, switch on LocoLED
                bitWrite(svtable.svt.pincfg[n].value2,4, IO_status[n-8]);                                 // Give .value2 the status of input [n] because the next LocoNet.send (OPC_INPUT_REP.... function needs this information
                LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);  // Send the input [n] change to LocoNet 
              }
          }
       
       if ((IO_timing[n-8] > 0) && ((millis() - IO_timing[n-8]) > WaitTime))                              // Did we wait longer then the WaitTime ?            
       {
          IO_timing[n-8] = 0;                                                                             // reset WaitTime
          IO_status[n-8] = 0;                                                                             // make input (hall-sensor) inactive after WaitTime
          LocoNet_communication (1);                                                                      // LocoNet communication, switch on LocoLED
          bitWrite(svtable.svt.pincfg[n].value2,4, IO_status[n-8]);                                       // Give .value2 the status of input [n] because the next LocoNet.send (OPC_INPUT_REP.... function needs this information
          LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);        // Send the input [n] change to LocoNet  
       }
    }                                                                                                     // This code is to avoid too many messages over LocoNet


// ********************************** RFID ***************************************

  unsigned char i=0, j=0;
   
//    SendPacketSensor.data[0]    =  0xE4; //OPC - variable length message 
//    SendPacketSensor.data[1]    =  Length of the message,  14 bytes 
//    SendPacketSensor.data[2]    =  0x41; //report type 
//    SendPacketSensor.data[3]    =  sensor address high
//    SendPacketSensor.data[4]    =  sensor address low 
//    SendPacketSensor.data[5-11] =  data bytes 4 .. 11, total 7 bytes
//    SendPacketSensor.data[12]   =  RFID-HI, all MSB bits of the data bytes
//    SendPacketSensor.data[13]   =  checksum byte of the RFID-7 report

// *****************************************
// Check the first RFID reader
// *****************************************
  if ( mfrc522_1.PICC_IsNewCardPresent() && mfrc522_1.PICC_ReadCardSerial())
  {  
     if ( ! compareUid( mfrc522_1.uid.uidByte, oldUid[0], mfrc522_1.uid.size))                            // if the card-ID differs from the previous card-ID
      {        
          setMessageHeader(0);                                                                            // Fill .data[0] till .data[4] with data.  Address is from the first hardware input
          
          SendPacketSensor.data[12]=0;                                                                    // clear the byte for the MS bits of the data bytes
          
          for(i=0, j=5; i< UID_LEN; i++, j++)                                                             // Fill SendPacketSensor.data with 7 data bytes information from de RFID card
          {
             if(mfrc522_1.uid.size > i)
             {
                SendPacketSensor.data[j] = mfrc522_1.uid.uidByte[i] & 0x7F;                                // LocoNet bytes have only 7 bits;
                                                                                                           // MSbit is transmited in the SendPacket.data[10]
                if(mfrc522_1.uid.uidByte[i] & 0x80)                                                        // if there is a MSB-bit in the data-byte ?
                {                                                       
                   SendPacketSensor.data[12] |= 1 << i;                                                    // bring this MSB-bit to the last databyte [12],  together with the other MSB-bit's 
                }
             } else {
                SendPacketSensor.data[j] = 0;
             }        
          }           // for(i=0, j=5; i< UID_LEN; i++, j++)
  
          
          SendPacketSensor.data[13]=0xFF;                                                                   // Reset the checksum data byte 
          
          for(j=0; j<13;j++)
            {
              SendPacketSensor.data[13] ^= SendPacketSensor.data[j];                                        // calculate the checksum for header + data of the RFID-7 message
            }   
          
          #ifdef DEBUG                             
            if(bSerialOk){
               Serial.print(F("LN send mess:"));
               dump_byte_array(SendPacketSensor.data, 14);
               Serial.println();
            }
          #endif
          
          LocoNet_communication (1);                                                                        // LocoNet communication, switch on LocoLED
          LocoNet.send( &SendPacketSensor, 14 );                                                            // Total RFID information is 14 bytes
  
          copyUid(mfrc522_1.uid.uidByte, oldUid[0], mfrc522_1.uid.size);                                    // fill oldUid with the latest card-ID    
          ResetUid_1 = millis() + 2000;                                                                     // record the time, after 2 seconds the main loop() releases the blockade on the same rfid ID
      
       
          // Halt PICC
          mfrc522_1.PICC_HaltA();
          // Stop encryption on PCD
          mfrc522_1.PCD_StopCrypto1();
      }
  } //if ( mfrc522_1.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()){


// *****************************************
// Check the second RFID reader
// *****************************************

  if ( mfrc522_2.PICC_IsNewCardPresent() && mfrc522_2.PICC_ReadCardSerial())
  {  
     if ( ! compareUid( mfrc522_2.uid.uidByte, oldUid[1], mfrc522_2.uid.size))                             // if the card-ID differs from the previous card-ID
      {        
          setMessageHeader(1);                                                                             // Fill .data[0] till .data[4] with data. Address is from the second hardware input
          
          SendPacketSensor.data[12]=0;                                                                     // clear the byte for the MS bits of the data bytes
          
          for(i=0, j=5; i< UID_LEN; i++, j++)                                                              // Fill SendPacketSensor.data with 7 data bytes information from de RFID card
          {
             if(mfrc522_2.uid.size > i)
             {
                SendPacketSensor.data[j] = mfrc522_2.uid.uidByte[i] & 0x7F;                                // LocoNet bytes have only 7 bits;
                                                                                                           // MSbit is transmited in the SendPacket.data[10]
                if(mfrc522_2.uid.uidByte[i] & 0x80)                                                        // if there is a MSB-bit in the data-byte ?
                {                                                       
                   SendPacketSensor.data[12] |= 1 << i;                                                    // bring this MSB-bit to the last databyte [12],  together with the other MSB-bit's 
                }
             } else {
                SendPacketSensor.data[j] = 0;
             }        
          }           // for(i=0, j=5; i< UID_LEN; i++, j++)
  
          
          SendPacketSensor.data[13]=0xFF;                                                                   // Reset the checksum data byte 
          
          for(j=0; j<13;j++)
            {
              SendPacketSensor.data[13] ^= SendPacketSensor.data[j];                                        // calculate the checksum for header + data of the RFID-7 message
            }   
          #ifdef DEBUG                             
            if(bSerialOk){
               Serial.print(F("LN send mess:"));
               dump_byte_array(SendPacketSensor.data, 14);
               Serial.println();
            }
          #endif 
          
          LocoNet_communication (1);                                                                        // LocoNet communication, switch on LocoLED
          LocoNet.send( &SendPacketSensor, 14 );                                                            // Total RFID information is 14 bytes
          
          copyUid(mfrc522_2.uid.uidByte, oldUid[1], mfrc522_2.uid.size);                                    // fill oldUid with the latest card-ID    
          ResetUid_2 = millis() + 2000;                                                                     // record the time, after 2 seconds the main loop() releases the blockade on the same rfid ID
      
       
          // Halt PICC
          mfrc522_2.PICC_HaltA();
          // Stop encryption on PCD
          mfrc522_2.PCD_StopCrypto1();
      }
  } //if ( mfrc522_1.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()){

  if (ResetUid_1 < millis()) 
    {
      oldUid[0][0] = 0;                                                                // Relaese, after 2 seconds, the blockade on the same rfid ID for Reader 1
      oldUid[0][1] = 0;                                                                // Twice a zero never happends
    }
    
 
  if (ResetUid_2 < millis()) 
    {
      oldUid[1][0] = 0;                                                                // Relaese, after 2 seconds, the blockade on the same rfid ID for Reader 2
      oldUid[1][1] = 0;                                                                // Twice a zero never happends
    }

} // end of loop()


// ********************************** RFID methods ***************************************

//***************************************************************************************************************************
// * Function : void setMessageHeader(uint8_t port)
// * This function calculates the adrr.low and addr.high byte, needed for the RFID-7 report to Rocrail.
// * First the sofware address of the specific port must be calculated with the CV's of this port.
// **************************************************************************************************************************
void setMessageHeader(uint8_t port)
{ 
  uint8_t odd_even;
  uint16_t iSenAddr;
    
  SendPacketSensor.data[0] = 0xE4;                          //OPC - variable length message 
  SendPacketSensor.data[1] = 14;                            //14 bytes length
  SendPacketSensor.data[2] = 0x41;                          //report type  

  if   (bitRead (svtable.svt.pincfg[port].value2, 5)) odd_even = 2;                                                       // bitread for bit 5 on SV5, SV8, SV11, SV14 etc.
  else odd_even = 1; 
       
  iSenAddr = (((svtable.svt.pincfg[port].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[port].value1 << 1 ) + odd_even);     // Caluculated software address of the port. For Port 1 .value1 == SV4 and .value2 == SV5
  //           (SV5 & 0x0F) << 8 == high byte                  +  SV4 << 1 == low byte                   + odd_even       == software-address of the hardware-port
  
   // the contents of the variable iSenAddr is the complete sensor address.
   // Now we must send this Address to LocoNet but we must split the address again in a high- and low part
   SendPacketSensor.data[3] = iSenAddr >> 7;                                                                              // addr.high for RFID-7 report to Rocrail
   SendPacketSensor.data[4] = iSenAddr & 0x7F;                                                                            // addr.low  for RFID-7 report to Rocrail    
   #ifdef DEBUG
    Serial.print ("iSenAddr == ");  Serial.println (iSenAddr); 
    Serial.print ("Addr_high == "); Serial.println (iSenAddr >> 7);   
    Serial.print ("Addr_low   == ");  Serial.println (iSenAddr & 0x7F);                            
   #endif
}

// ********************************** LocoIO methods ***************************************

/*********************************************************************************************************************
* Function    : void notifySensor( uint16_t Address, uint8_t State )
* Discription : This call-back function is called from LocoNet.processSwitchSensorMessage for all Sensor messages
*               In the LocoNet.processSwitchSensorMessage is a pointer to this function
*               The pointer is actualy the name of this function
**********************************************************************************************************************/
void notifySensor( uint16_t Address, uint8_t State )
{
  {
    #ifdef DEBUG
    Serial.print("Sensor: ");
    Serial.print(Address, DEC);
    Serial.print(" - ");
    Serial.println( State ? "Active" : "Inactive" );
    #endif
  }
}

/*********************************************************************************************************************
* Function    : void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
* Discription : This call-back function is called from LocoNet.processSwitchSensorMessage
                for all Switch Request messages
                In the LocoNet.processSwitchSensorMessage is a pointer to this function
                The pointer is actualy the name of this function
                This function gives you allways two commands within 0,5 seconds.
                In both commands the address and Direction are the same, only Ouput switches from 1 to 0 within this 0,5 second
                This behavior prevents the turnout coil to become too hot.
                In this function we only deal with steady state outputs, so we only use the vaiabele Direction and Address
**********************************************************************************************************************/
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int n, i, temp_value1;

   LocoNet_communication (1);                                                                        // LocoNet communication, switch on LocoLED
    
  if (Direction > 0) Direction=1;  else Direction = 0;
  
  for (n=0; n<16; n++)                                                                               // Check if the Address is assigned, configured as output and same Direction
  {      
    if ((Software_address[n]  == Address) &&  (bitRead(svtable.svt.pincfg[n].cnfg,7) == 1))          // Address ok and ...Setup as an Output ?
        {
          #ifdef DEBUG 
          Serial.print ("aangeleverd adres : "); Serial.println(Address, DEC);
          Serial.print ("opgehaald adres   : "); Serial.println(Software_address[n], DEC);
          #endif
          break;                                                                                     // found the hardware port !
        }
  }
                                               
    if (Direction) digitalWrite(pinMap[n-8], HIGH);        // array Software_address[16] and pinMap[8] are working together. Software_address[16] has the software adresse of all 16 ports
    else           digitalWrite(pinMap[n-8], LOW);         // pinMap[16] has all the 16 hardware pin numbers  
}  

/*********************************************************************************************************************
* Function    : void SwitchPulseContact_OFF (int contact)
* Discription : 
**********************************************************************************************************************/


boolean processPeerPacket()                                                                             //Check is a OPC_PEER_XFER message
{
  
  if (LnPacket->px.command != OPC_PEER_XFER) return(false);

  if ((LnPacket->px.dst_l!=0 || LnPacket->px.d5!=0) &&                                                  //Check is my destination
      (LnPacket->px.dst_l!=0x7f || LnPacket->px.d5!=svtable.svt.addr_high) &&
      (LnPacket->px.dst_l!=svtable.svt.addr_low || LnPacket->px.d5!=svtable.svt.addr_high))
    return(false);
    
  bitWrite(LnPacket->px.d1,7,bitRead(LnPacket->px.pxct1,0));                                            //Set high bits in right position
  bitWrite(LnPacket->px.d2,7,bitRead(LnPacket->px.pxct1,1));
  bitWrite(LnPacket->px.d3,7,bitRead(LnPacket->px.pxct1,2));
  bitWrite(LnPacket->px.d4,7,bitRead(LnPacket->px.pxct1,3));
  
  bitWrite(LnPacket->px.d5,7,bitRead(LnPacket->px.pxct2,0));
  bitWrite(LnPacket->px.d6,7,bitRead(LnPacket->px.pxct2,1));
  bitWrite(LnPacket->px.d7,7,bitRead(LnPacket->px.pxct2,2));
  bitWrite(LnPacket->px.d8,7,bitRead(LnPacket->px.pxct2,3));

                                                                                                        
  if (LnPacket->px.d1==2)                                                                                //OPC_PEER_XFER D1 -> Command (1 SV write, 2 SV read)
                                                                                                         //OPC_PEER_XFER D2 -> Register to read or write
  {
    sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2+1], svtable.data[LnPacket->px.d2+2]);
    return (true);
  }
  
  
  if (LnPacket->px.d1==1)                                                                                 //Write command
  {
    
    if (LnPacket->px.d2>0)                                                                                //SV 0 contains the program version (write SV0 == RESET? )
    {    
      svtable.data[LnPacket->px.d2]=LnPacket->px.d4;                                                      //Store data
      EEPROM.write(LnPacket->px.d2,LnPacket->px.d4);
      
      #ifdef DEBUG
      Serial.print("WRITE "); Serial.print(LnPacket->px.d2); Serial.print(" <== ");
      Serial.print(LnPacket->px.d4); Serial.print(" | ");
      Serial.print(LnPacket->px.d4, HEX); Serial.print(" | ");
      Serial.println(LnPacket->px.d4, BIN);
      #endif
    }       
    LocoNet_communication (1);                                                                            // LocoNet communication, switch on LocoLED
    sendPeerPacket(0x00, 0x00, LnPacket->px.d4);                                                          // Answer packet 
    return (true);
  }
  
  return (false);
  
}

void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2)
{
  lnMsg txPacket;

  txPacket.px.command=OPC_PEER_XFER;
  txPacket.px.mesg_size=0x10;
  txPacket.px.src=svtable.svt.addr_low;
  txPacket.px.dst_l=LnPacket->px.src;
  txPacket.px.dst_h=LnPacket->px.dst_h; 
  txPacket.px.pxct1=0x00;
  txPacket.px.d1=LnPacket->px.d1;                         //Original command
  txPacket.px.d2=LnPacket->px.d2;                         //SV requested
  txPacket.px.d3=svtable.svt.vrsion;
  txPacket.px.d4=0x00;
  txPacket.px.pxct2=0x00;
  txPacket.px.d5=svtable.svt.addr_high;                   //SOURCE high address
  txPacket.px.d6=p0;
  txPacket.px.d7=p1;
  txPacket.px.d8=p2;

  //Set high bits in right position  
  bitWrite(txPacket.px.pxct1,0,bitRead(txPacket.px.d1,7));
  bitClear(txPacket.px.d1,7);
  bitWrite(txPacket.px.pxct1,1,bitRead(txPacket.px.d2,7));
  bitClear(txPacket.px.d2,7);
  bitWrite(txPacket.px.pxct1,2,bitRead(txPacket.px.d3,7));
  bitClear(txPacket.px.d3,7);
  bitWrite(txPacket.px.pxct1,3,bitRead(txPacket.px.d4,7));
  bitClear(txPacket.px.d4,7);
  bitWrite(txPacket.px.pxct2,0,bitRead(txPacket.px.d5,7));
  bitClear(txPacket.px.d5,7);
  bitWrite(txPacket.px.pxct2,1,bitRead(txPacket.px.d6,7));
  bitClear(txPacket.px.d6,7);
  bitWrite(txPacket.px.pxct2,2,bitRead(txPacket.px.d7,7));
  bitClear(txPacket.px.d7,7);
  bitWrite(txPacket.px.pxct2,3,bitRead(txPacket.px.d8,7));
  bitClear(txPacket.px.d8,7);

  LocoNet_communication (1);        // LocoNet communication, switch on LocoLED
  LocoNet.send(&txPacket);
  
  #ifdef DEBUG
  Serial.println("Packet sent!");
  #endif
}
