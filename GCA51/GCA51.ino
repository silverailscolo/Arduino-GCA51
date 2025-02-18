/**************************************************************************
   LocoGCA51 - Configurable Arduino LocoNet + RFID Module
   Copyright (C) 2016 Gerard Remmerswaal
   Copyright (C) 2025 Egbert Broerse

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
  7 -> LocoNet TX
  8 -> LocoNet RX
  9,10,11,12,13 -> Configurable I/O from 6 to 10
  A0,A1,A2,A3,A4,A5-> Configurable I/O from 11 to 16
  ------------------------------------------------------------------------
  CREDITS:
  Based on MRRwA LocoNet libraries for Arduino - http://mrrwa.org/ and
  the LocoNet Monitor example.
  Inspired in GCA50 board from Peter Giling - http://www.phgiling.net/
  Idea also inspired in LocoShield from SPCoast - http://www.scuba.net/
  Thanks also to Rocrail group - http://www.rocrail.org
  Thanks to the LocoNet part of software from Dani Guisado/ClubNCaldes
  ------------------------------------------------------------------------
  ISSUES
  sensor address explodes after 10 reads/LN_BUFF_LEN==10
*************************************************************************/

#include "LocoNet.h"
#include <SPI.h>
#include <EEPROM.h>
// LocoIO functions from GCA51a included as extern, see below
#include "rfid2ln.h"
#include <MFRC522.h>
#include <Arduino.h>

#define VERSION       151                      // 106 for LocoIO functions, must be type int
#define DEBUG                                  // Uncomment this line to debug through the serial monitor
//#define JMRI4                                  // Uncomment this line to send Lissy IR messages instead of Lissy RFID-7
#define LN_TX_PIN       7                      // Arduino Pin used as LocoNet Tx; Rx Pin is always the ICP Pin
#define RST_PIN         6                      // Arduino Pin used as ResetPowerDownPin
#define SDA_1_PIN      10                      // The SDA_1 pin provides the Arduino with tag data from Reader 1
#define SDA_2_PIN       9                      // The SDA_2 pin provides the Arduino with tag data from Reader 2
#define LocoLED         4                      // LocoLED lights up when there is LocoNet communication
#define LocoLED_wait  200                      // LocoLED is always on for 200 msec
#define PulseTime     300                      // PulseTime for all the Pulse Outputs in msec. - TODO board config SV0 bit y
#define WaitTime      500                      // Wait Time for all block Inputs in msec.
#define FlashTime     250                      // Frequency of flasher - TODO board config SV0 bit x

uint8_t ucBoardAddrHi = 1;                     // board address high; always 1 because GCA51 has no room for high byte in RFID-7 message
uint8_t ucBoardAddrLo = 88;                    // board address low; default 88

//uint8_t NR_OF_RFID_PORTS = 2;                // GCA51, same as default set in rfid2ln lib, but override here for future new hardware?
unsigned long resetUid[NR_OF_RFID_PORTS];      // stores the timestamp when the old UID from Reader[i] was stored

MFRC522 mfrc522[NR_OF_RFID_PORTS];
#if NR_OF_RFID_PORTS == 1
uint8_t boardVer[] = "RFID2LN Vxx SINGLE";
#elif NR_OF_RFID_PORTS == 2
uint8_t boardVer[] = "RFID2LN Vxx MULTI";
#endif
char verLen = sizeof(boardVer);
#define INFORMATPOWERON

#define ELEMENTCOUNT(x) (sizeof(x) / sizeof(int)) // (sizeof(x[0]) / sizeof(int)

LocoNetSystemVariableClass sv;
lnMsg *LnPacket;                               // pointer to lnMsg
lnMsg SendPacketSensor[LN_BUFF_LEN];           // SendPacketSensor is now a uint8_t data[16] array. Must set bounds in Arduino 1.8.19

uint8_t uiLnSendCheckSumIdx = 13;
#ifdef JMRI4
uint8_t uiLnSendLength = 8;  // 8 bytes legacy LISSY IR format
#else
uint8_t uiLnSendLength = 14; // 14 bytes RFID-7 as in v150
#endif
uint8_t uiLnSendMsbIdx = 12;
uint8_t uiStartChkSen;

uint8_t oldUid[NR_OF_RFID_PORTS][UID_LEN];     // 7 bytes of information from the previous tag seen by each RFID Reader

boolean bSerialOk = false;

byte mfrc522Cs[] = {SDA_1_PIN, SDA_2_PIN};

uint8_t uiBufWrIdx = 0;
uint8_t uiBufRdIdx = 0;
uint8_t uiBufCnt = 0;

boolean bUpdateOutputs = false; // TODO update from SV0 bit?

uint8_t uiRfidPort = 0;

uint8_t uiNrEmptyReads[NR_OF_RFID_PORTS];
boolean bSensorActive[NR_OF_RFID_PORTS];

uint8_t uiActReaders = 0;
uint8_t uiFirstReaderIdx = 0;

// Arduino Nano pin assignment to each of the 8 free outputs
// Actually the Arduino has only 14 digital IO (0 till 13). You can however use the analog inputs 0 - 5 also as digital IO.
// To change Analog input 0 to a digital IO you should use pin number 14 ==>  pinMode (14, OUTPUT);
// The second Analog input will give you pinMode 15 etc. The GCA185 board uses all the available IO pins in this way.
// To connect two RFID-RC522 sensors to the GCA185/GCA51 you need the Arduino pin numbers 2, 10, 11, 12 and 13, so you cannot use these pins for another purpose.
// The LocoNet pins, needed for the communication, are the Arduino Nano pin numbers 7 and 8

byte     pinMap[8] = {14, 15, 16, 17, 18, 19, 2, 3}; // The analog inputs are used as digital I/O and that is why the names of these ports are changed.
// So, A0 == D14,  A1 == D15,  A2 == D16, ....A7 == D21. 2 and 3 are D2 and D3. These two I/O use INT0 and INT1
uint16_t softwareAddress[16];                    // composite software address of all the hardware ports (3-8 not used on GCA51)
volatile byte IO_status[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // remember the last I/O status. '1' means port is Active. '0' means port is Inactive

// 3 bytes defining a pin behavior and software address of the hardware ports ( http://wiki.rocrail.net/doku.php?id=loconet-io-en )
struct PIN_CFG
{
  uint8_t cnfg;
  uint8_t value1;
  uint8_t value2;
};

// Memory map exchanged with SV read and write commands ( http://wiki.rocrail.net/doku.php?id=lnsv-en )
struct SV_TABLE
{
  uint8_t vrsion;
  uint8_t addr_low;
  uint8_t addr_high;
  PIN_CFG pincfg[16];                    // pincfg[16] has all the hardware / software settings of the 8 I/O ports: cnfg, value1 and value2
};

union SV_DATA                            // Union to access the data with the struct or by index
{
  SV_TABLE svt;
  uint8_t data[51];
};
SV_DATA svtable;                         // Union declaration svtable

const uint8_t rfidOptions[2] = {27, 31};

typedef struct CNFG_OPTIONS
{
  uint8_t code;
  char *description;  // max length of description = 10 in getConfig(i)
};

const CNFG_OPTIONS configOptions[16] = {
  { .code = 15, .description = "tggl",},             // [0] inputs:
  { .code = 31, .description = "blck",},             // [1]
  { .code = 91, .description = "blck del",},         // [2]
  { .code = 39, .description = "btn ind",},          // [3]
  { .code = 47, .description = "btn",},              // [4]
  { .code = 23, .description = "to",},               // [5] single contact "normal" turnout feedback
  { .code = 55, .description = "sw ct",},            // [6] 2 contacts turnout feedback, for 2: .value2 bits 4-7 = 3
  { .code = 27, .description = "unused",},           // [7] outputs:
  { .code = 128, .description = "off",},             // [8] for 1: .value2 bits 4-7 (JMRI HDL LocoIO Value2A) = 1
  { .code = 129, .description = "on",},              // [9] for 2: .value2 bits 4-7 = 3
  { .code = 136, .description = "pls sft",},         // [10]
  { .code = 140, .description = "pls hrd",},         // [11]
  { .code = 144, .description = "off*",},            // [12]
  { .code = 145, .description = "on*",},             // [13]
  { .code = 192, .description = "blck",},            // [14]
  { .code = 208, .description = "blck*",},           // [15]
};

// Timers for each input configured as "delayed"
// inputs defined as "delayed" will keep the signal high at least 2 seconds (why 2s? LocoIO docs says: 1-2*blinkDuration)
unsigned long inpTimer[16];            // block delay per cnfg port
uint8_t blinkRate = 0;                 // default board setting for blinking rate of output ports
uint16_t blinkDuration = 1000;
unsigned long currentBlinkMillis = 0;  // use the same time for all LED flashes to keep them synchronized
unsigned long previousBlinkMillis = 0; // last time LED changed state
uint8_t blinkState[16];

// other board config options
boolean alternateMode = false;
boolean portRefresh = false;          // TODO send update of all input states when SV0 is written

// functions common with GCA51a LocoIO are in GCA51Func.cpp
// extern boolean processPeerPacket();
// extern void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2); // added LED blink so must use a local copy of GCA50a method
// extern void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction); // idem

MFRC522::MIFARE_Key key;

// ********************************** Utility methods ***************************************

/*********************************************************************************
    Purpose: signal LocoNet activity on Nano
    Turn LocoLED on  when LocoNet communication starts
    Turn LocoLED off when the waiting time has elapsed
 *********************************************************************************/
void LocoNet_communication(byte on_off)
{
  static unsigned long LED_on_off;

  if (on_off == 1)                        // LocoLED is off
  {
    digitalWrite (LocoLED, HIGH);         // to signal LocoNet communication has started, turn on LocoLED
    LED_on_off = millis();                // remember start time
  }

  if ((LED_on_off + LocoLED_wait) < millis())     // if the wait time has expired,
  {
    LED_on_off = 0;
    digitalWrite (LocoLED, LOW);                  // turn off LocoLED
  }
}

/*********************************************************************************
    Purpose: calculate the software addresses and store them in global variable softwareAddress[16]

    Compare to GCA50a myAddress[],renamed to softwareAddress[] since v1.07 2025
 *********************************************************************************/
void CalculateAddress()
{
  uint8_t n; // used for lookup
  byte odd_even;

  // RFID ports
  for (n = 0; n < 2; n++)
  {
    if ((svtable.svt.pincfg[n].cnfg == rfidOptions[0]) || (svtable.svt.pincfg[n].cnfg == rfidOptions[1]))
      // declared as input, active low: see configOptions[] for descriptions
    {
      if (bitRead (svtable.svt.pincfg[n].value2, 5)) odd_even = 2; // bitread for bit 5 in SV5, SV8, SV11, SV14 etc.
      else odd_even = 1;
      softwareAddress[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1 << 1 ) + odd_even); // Calculate software address of port. Eg. for Port 1 .value1 == SV4 and .value2 == SV5
      // (SV5 & 0x0F) << 8 == high byte + SV4 << 1 == low byte + odd_even == software-address of the hardware-port
      Serial.print ("- RFID Reader RC522-"); Serial.print (n + 1); Serial.print (" input, address: "); Serial.print(softwareAddress[n], DEC);
      Serial.print(" (cfg: "); Serial.print(svtable.svt.pincfg[n].cnfg); Serial.println(")");
    } else {
      Serial.print ("- RFID Reader port"); Serial.print (n); Serial.print(" should be configured as Input - Block Detector - Active Low (- Delayed optional). Skipping. Err: config="); Serial.println(svtable.svt.pincfg[n].cnfg);
    }
  }

  // normal I/O ports
  for (n = 2; n < 16; n++)
  {
    if (findConfig(svtable.svt.pincfg[n].cnfg) != -1)
    {
      if (!bitRead(svtable.svt.pincfg[n].cnfg, 7))
        // configured as inputs, active low
      {
        if (bitRead (svtable.svt.pincfg[n].value2, 5)) odd_even = 2; // bitread for bit 5 in SV5, SV8, SV11, SV14 etc.
        else odd_even = 1;

        softwareAddress[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1 << 1 ) + odd_even); // Calculate software address of port. For Port 1 .value1 == SV4 and .value2 == SV5
        // (SV5 & 0x0F) << 8 == high byte + SV4 << 1 == low byte + odd_even == software-address of the hardware-port
        if (n > 7) {
          Serial.print("- Port "); Serial.print(n); Serial.print (" [H"); Serial.print (n - 7); Serial.print("] input, address: "); Serial.print(softwareAddress[n], DEC);
          Serial.print(" (cfg: ");
          Serial.print(svtable.svt.pincfg[n].cnfg);
          Serial.print(" ");
          Serial.print(getConfig(svtable.svt.pincfg[n].cnfg)); // adds pin config description
          Serial.println(")");
        } else {
          Serial.print("- Port "); Serial.print(n); Serial.println(" N/A"); // only used on GCA51
        }
      }
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 7))
        // configured as outputs
      {
        softwareAddress[n] = (((svtable.svt.pincfg[n].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[n].value1) + 1); // Calculated software address of the port. Eg. for Port 1 .value1 == SV4 and .value2 == SV5
        if (n > 7)
        {
          Serial.print ("- Port "); Serial.print (n); Serial.print (" [H"); Serial.print(n - 7); Serial.print("] output, address: "); Serial.print(softwareAddress[n], DEC);
          Serial.print(" (cfg: ");
          Serial.print(svtable.svt.pincfg[n].cnfg);
          Serial.print(" ");
          Serial.print(getConfig(svtable.svt.pincfg[n].cnfg)); // adds pin config description
          // add no. 1/2 output pair = .value2 bits 4-7
          int logic = svtable.svt.pincfg[n].value2 & 0xF0;
          if (logic == 3)
          {
            Serial.print(" 2 ");
          }
          else if (logic == 1)
          {
            Serial.print(" 1 ");
          }
          Serial.println(")");
        } else {
          Serial.print ("- Port "); Serial.print (n); Serial.println(" N/A");
        }
      }
    }
    else
    {
      Serial.print ("Port"); Serial.print (n); Serial.print(" has an unknown setting. Err: cnfg="); Serial.println(svtable.svt.pincfg[n].cnfg);
    }
  }
}

/***************************************************************************************************************************
  Purpose: assign interrupts to input pins
  This function is the last function called in de setup() routine when all the IO's have their function, input or output.
   Only the inputs will get assigned an interrupt
****************************************************************************************************************************/
void InitialiseInterrupt()
{
  int n;
  cli();                    // turn off interrupts while messing with their settings
  PCICR = 0x02;             // PCIE1 interrupt is enabled for pin group A0 .. A7

  for (n = 8; n < 14; n++)  // only inputs should give an interrupt. Select A0..A5
  {
    if (!bitRead(svtable.svt.pincfg[n].cnfg, 7))        // I/O port is an input
      bitWrite (PCMSK1, n - 8, 1);                      // interrupts on pins A0 .. A5
  }

  if (bitRead(svtable.svt.pincfg[14].cnfg, 7) == 0) bitWrite (EIMSK, 0, 1);  // if D2 is an input ==> enable INT0
  if (bitRead(svtable.svt.pincfg[15].cnfg, 7) == 0) bitWrite (EIMSK, 1, 1);  // if D3 is an input ==> enable INT1

  EICRA  = 0b00001010;      // D2 INT0 on falling Edge.    D3 INT1 on falling Edge.
  sei();                    // turn interrupts back on
}

ISR(INT0_vect)              // ISR(INT0_vect) only reacts on a descending edge at the input
{
  IO_status[6] = 1;
}       //Serial.println("D2");}

ISR(INT1_vect)              // ISR(INT1_vect) only reacts on a descending edge at the input
{
  IO_status[7] = 1;
}       //Serial.println("D3");}

ISR(PCINT1_vect)            // Interrupt service routine. Every single PCINT8..14 (=ADC0..5) change will generate an interrupt: but this will always be the same interrupt routine
{
  if (digitalRead(A0) == 0)  IO_status[0] = 1;  //Serial.println("A0");
  if (digitalRead(A1) == 0)  IO_status[1] = 1;  //Serial.println("A1");
  if (digitalRead(A2) == 0)  IO_status[2] = 1;  //Serial.println("A2");
  if (digitalRead(A3) == 0)  IO_status[3] = 1;  //Serial.println("A3");
  if (digitalRead(A4) == 0)  IO_status[4] = 1;  //Serial.println("A4");
  if (digitalRead(A5) == 0)  IO_status[5] = 1;  //Serial.println("A5");
}

/***************************************************************************************************************************
  Purpose: to search a list of operators for a match on the target operator
  Parameter list :
  char target     the pin config key to find
  Return value :
  int             the index for a match, -1 no match
****************************************************************************************************************************/
int findConfig(int target)
{
  uint8_t i;
  for (i = 0; i < ELEMENTCOUNT(configOptions); i++)
  {
    if (configOptions[i].code == target)
    {
      return i;
    }
  }
  return -1;
}

/***************************************************************************************************************************
  Purpose: Display all pin descriptions (for Serial Monitor)
  ***********************/
char *getConfig(int pin)
{
  const char *desc = (char*) malloc (10);
  if (findConfig(svtable.svt.pincfg[pin].cnfg) != -1) {
    strcpy(desc, configOptions[uint8_t(svtable.svt.pincfg[pin].cnfg)].description);
    return desc;
  } else {
    return ("not found");
  }
}


// *************** SETUP *************************

void setup()
{
  uint32_t uiStartTimer;
  uint16_t uiElapsedDelay;
  uint16_t uiSerialOKDelay = 5000;
  int i, n;
  pinMode (LocoLED, OUTPUT);                    // LocoLED pin to indicate LocoNet communication

  // start_setup();  // Start values of the board in LocoGCA51.cpp <<<< Not available, copied from latest GCA50a
  // rfid2ln boardSetup() assumes 1 RFID reader per board so we can't use it here TODO write GCA51 setup() method

  // Configure the serial port
#ifdef DEBUG
  Serial.begin(9600); // Initialize serial communications with the PC (old bootloader baud or monitor garbage after flashing; on a new Nano use 115200 bd)
  Serial.print("GCA51 v."); Serial.println(VERSION);
#ifdef JMRI4
  Serial.println("Firmware set up to send JMRI 4.22-5.12 compatible LISSY IR messages (only 2 RFID tag bytes used).");
#endif
  uiStartTimer = millis();
  do { // wait for the serial interface, max 1 second.
    uiElapsedDelay = millis() - uiStartTimer;
  } while ((!Serial) && (uiElapsedDelay < uiSerialOKDelay)); // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  if (Serial) { // serial interface OK
    bSerialOk = true;
    Serial.println(F("************************************************"));
  }
#endif

  // Initialize the LocoNet interface
  LocoNet.init(LN_TX_PIN); // Use explicit naming of the Tx Pin to avoid confusion
  sv.init(MANUF_ID, BOARD_TYPE, 1, 1); // to see if needed just once (saved in EEPROM)

  // Check for a valid config
  if (svtable.svt.vrsion != VERSION || svtable.svt.addr_low < 1 || svtable.svt.addr_low > 240 || svtable.svt.addr_high < 1 || svtable.svt.addr_high > 100 )
  {
    svtable.svt.vrsion = VERSION;
    svtable.svt.addr_low = ucBoardAddrLo;
    svtable.svt.addr_high = ucBoardAddrHi;
    EEPROM.write(100, VERSION);
    EEPROM.write(1, svtable.svt.addr_low);
    EEPROM.write(2, svtable.svt.addr_high);
  }
  Serial.print("Module "); Serial.print(svtable.svt.addr_low); Serial.print("/"); Serial.println(svtable.svt.addr_high);

  // Load config from EEPROM
  for (n = 0; n < 51; n++) {
    svtable.data[n] = EEPROM.read(n);  // Read the values of SV0 till SV51. The values in EEPROM were OK or standardised in start_setup()
  }

  CalculateAddress();                  // Calculate software addresses of pins and store in global variable softwareAddress[16]. Also prints config to Console

  // load board settings from SV0
  // from Public_Domain_HDL_LocoIO definition:
  //      <variable CV="0" mask="VVVVXXXX" item="Blink Rate" default="0"> DONE, see blinkRate
  //      <label>Blink Rate:</label> 0=slow to 15=fast
  //
  //      <variable CV="0" mask="XXXVXXXX" item="Board Active High" default="0"> ALWAYS ACTIVE LOW - NO CONFIG
  //        <tooltip>Default: unselected = Active Low</tooltip>
  //
  //      <variable CV="0" mask="XXXXVVXX" item="Action Mode" default="0"> NOT USED - ALWAYS 0
  //
  //      <variable CV="0" mask="XXXXXXVX" item="Alternate Mode" default="0">
  //        0=Fixed 1=Alternating
  //        <tooltip>Button sends alternating or fixed code</tooltip>
  //
  //      <variable CV="0" mask="XXXXXXXV" item="Port Refresh" default="0">

  blinkRate = (svtable.data[0] >> 4);  // actual blinkPeriod was matched to an HDL LocoIO
  blinkDuration = 1000 - 30 * blinkRate; // use 50% of blinkPeriod. See also FlashTime const
  Serial.print("Board blink rate: "); Serial.print(blinkRate); Serial.print( " blink period: "); Serial.print(blinkDuration * 2); Serial.println("ms");

  alternateMode = svtable.data[0] & 0x2;
  portRefresh = svtable.data[0] & 0x1;

  Serial.println("LocoIO functions compatible to v148/149");

  // Configure I/O pins and give the outputs a start value
#ifdef DEBUG
  Serial.println("Initializing pins...");
#endif
  for (n = 8; n < 16; n++)            // The first 8 I/O ports are already set and are not available to users, except to set addresses of ports 1 and 2 (RFID sensor ports)
    // The actual hardware Nano pin numbers are declared in the global variable pinMap[]
  {
    if (bitRead(svtable.svt.pincfg[n].cnfg, 7))                                         // if cnfg bit 7 == 1, pin is an Output
    {
      pinMode(pinMap[n - 8], OUTPUT);
      if (bitRead(svtable.svt.pincfg[n].cnfg, 0)) digitalWrite(pinMap[n - 8], HIGH);    // if cnfg bit 0 == 1 the output is HIGH at startup
      else digitalWrite(pinMap[n - 8], LOW);                                            // else the output is LOW at startup

      if (bitRead(svtable.svt.pincfg[n].cnfg, 4)) blinkState[n] = 1;                    // start blink as ON
      else blinkState[n] = 255; // uint8_t off marker


    }
    else                                                                                // if cnfg bit 7 is 0, pin is an Input
    {
      pinMode(pinMap[n - 8], INPUT_PULLUP);
      bitWrite(svtable.svt.pincfg[n].value2, 4, 1);  // block detector, no pulse contact
    }
    //inpTimer[n] = 1000; // TODO get the exact bit for each port from svtable.svt.data[n]

    InitialiseInterrupt();                           // (only) the inputs will get an interrupt
  }

  // ********************************** init RFID **********************************

  SPI.begin();                                       // Init SPI bus

#if USE_INTERRUPT
  regVal = 0xA0; //rx irq
#endif

  /*
     Only initialisation; all readers should be initialised before
     any communication
  */
  for (uint8_t i = 0; i < NR_OF_RFID_PORTS; i++) {
    mfrc522[i].PCD_Init(mfrc522Cs[i], RST_PIN);
  }

  /* Detect the active readers. If version read != 0xFF => reader active */
  for (uint8_t i = 0; i < NR_OF_RFID_PORTS; i++) {
    byte readReg = mfrc522[i].PCD_ReadRegister(mfrc522[i].VersionReg);

    if (bSerialOk) {
      Serial.print(F("RFID Reader [RC522-"));
      Serial.print(i + 1);
    }

    if ((readReg == 0x00) || (readReg == 0xFF)) { // reader missing
      if (bSerialOk) {
        Serial.println(F("] absent"));
      }
    } else {
      if (bSerialOk) {
        Serial.print(F("] present; version = "));
        Serial.println(readReg, HEX);
      }
      if (0 == uiActReaders) { // save the index of the first active reader
        uiFirstReaderIdx = i;
        uiRfidPort = uiFirstReaderIdx; // initialize the starting reader counter
      }
      uiActReaders++;
      //calcSenAddr(i); // stores rfid reader address (not the port) in ucAddrHiSen[i] and ucAddrLoSen[i] <<<<<<<<<<<<<< wrong arrays, replaced for GCA51

#if USE_INTERRUPT
      pinMode(mfrc522Irq[i], INPUT_PULLUP);

      /*
          Allow the ... irq to be propagated to the IRQ pin
          For test purposes propagate the IdleIrq and loAlert
      */
      mfrc522[i].PCD_WriteRegister(mfrc522[i].ComIEnReg, regVal);

      delay(10);
      attachInterrupt(digitalPinToInterrupt(mfrc522Irq[i]), readCard[i](), FALLING);
      bNewInt[i] = false;
#endif
    } //if(readReg)
  } //for(uint8_t i = 0

  if (bSerialOk) {
    Serial.print(F("Nr. of active RFID readers: "));
    Serial.println(uiActReaders);
    Serial.println(F("************************************************"));
  }

  // **************************** External Interrupts *****************************

} // end setup()


/************************ MAIN LOOP () *************************/
void loop()
{
  static uint8_t n;
  static int time_msec;
  byte temp_IO;
  static unsigned long IO_timing[8];                   // array[8] with Pulse- or Debounce timing for each IO-port
  static unsigned long CurrentTime;                    // time at this moment
  currentBlinkMillis = millis();                       // capture the latest value of millis()
  static byte remember_input[8];                       // remembers which input was active.  After "waittime" the program will reset this input(s)

  LocoNet_communication(0);                            // turn off LocoLED when the wait time has expired

  /***************************** Handle LocoNet Messages ********************************
     Address programming over LocoNet
     compare to GCA50a
  */

  LnPacket = LocoNet.receive();                        // Check for any received LocoNet packets
  if (LnPacket)
  {
#ifdef DEBUG
    if (bSerialOk) {
      Serial.print(F("Received LN mess. "));
      dump_byte_array(LnPacket->data, LnPacket->data[1]);
      Serial.println();
    }
#endif

    if (!LocoNet.processSwitchSensorMessage(LnPacket)) // check for PEER packet if this packet was not a Switch or Sensor Message
    {
      processPeerPacket();                             // from NCaldes GCA50a
    }
  }

  /********************************** OUTPUTS *******************************************
    handled by call-back function notifySwitchRequest to LocoNet.processSwitchSensorMessage
    blinking outputs are checked in the next for loop

   ******************************* HANDLE INPUTS *****************************************
    handles: (delayed) block detectors, toggles/buttons direct/indirect
    TODO switch point feedback contact1/contact 2, alternating code for bush buttons (board config)
  */
  for (n = 8; n < 16; n++)                                                                           // Check I/O ports 8 - 15. I/O ports 0 - 7 are used for the communication with the RFID equipment
  {
    updateBlink(n);                                                                                  // override pin state for blinking outputs

    if ((IO_timing[n - 8] == 0) && (!bitRead(svtable.svt.pincfg[n].cnfg, 7)))                        // no WaitTime active && port is an input
    {
      if (IO_status[n - 8] == 1)                                                                     // IO_status contains the last known value. At the ISR's this array is filled with new input information
      {
        IO_timing[n - 8] = millis();                                                                 // input is active, start timer
        LocoNet_communication(1);                                                                    // turn on LocoLED
        bitWrite(svtable.svt.pincfg[n].value2, 4, IO_status[n - 8]);                                 // Store state in input[n].value2 bit because the next LocoNet.send (OPC_INPUT_REP.... function needs this information

        if (svtable.svt.pincfg[n].cnfg & 0x7) // pushbutton/toggle input (15 or 39 or 47) or turnout feedback 1/2 (23 or 55)
        {
          if (svtable.svt.pincfg[n].cnfg & 0x8) // direct: OPC_SW_REQ (0xB0)
          {
            LocoNet.send(OPC_SW_REQ, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);     // Send the input[n] change to LocoNet
          }
          else // indirect: OPC_SW_REP (0xB1) 47
          {
            LocoNet.send(OPC_SW_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);     // Send the input[n] change to LocoNet
          }
        }
        else //if (svtable.svt.pincfg[n].cnfg & 0x1) // if block detector input active low (31 or 91) - default
        {
          LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);    // Send the input[n] change to LocoNet
        }
#ifdef DEBUG
        if (bSerialOk) {
          Serial.println(F("Sent out input as LN mess. "));
        }
#endif
      }
    }

    if ((IO_timing[n - 8] > 0) && ((millis() - IO_timing[n - 8]) > WaitTime))                         // Did we wait longer then the WaitTime?
    { // from LOCOIO docs: Remark: The switch off delay depends on the established blinking rate.
      IO_timing[n - 8] = 0;                                                                           // reset WaitTime
      IO_status[n - 8] = 0;                                                                           // make input (hall-sensor) inactive after WaitTime
      LocoNet_communication(1);                                                                       // turn on LocoLED
      bitWrite(svtable.svt.pincfg[n].value2, 4, IO_status[n - 8]);                                    // Store state in input[n].value2 bit because the next LocoNet.send (OPC_INPUT_REP.... function needs this information
      LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n].value1, svtable.svt.pincfg[n].value2);        // Send the input[n] change to LocoNet
    }
  }

  /********************************** HANDLE RFID ***************************************/
  unsigned char i = 0, j = 0;

  //    SendPacketSensor[i].data[0]    =  0xE4; //OPC - variable length message
  //    SendPacketSensor[i].data[1]    =  Length of the message,  14 bytes
  //    SendPacketSensor[i].data[2]    =  0x41; //report type
  //    SendPacketSensor[i].data[3]    =  sensor address high
  //    SendPacketSensor[i].data[4]    =  sensor address low
  //    SendPacketSensor[i].data[5-11] =  data bytes 4 .. 11, total 7 bytes
  //    SendPacketSensor[i].data[12]   =  RFID-HI, all MSB bits of the data bytes
  //    SendPacketSensor[i].data[13]   =  checksum byte of the RFID-7 report


  /********* Check the RFID readers *************/

  // v151 adds flexible loop from rfid2ln, renamed uiAddrSenFull[i] to softwareAddress[i]
  if (uiActReaders > 0) {
    if (uiBufCnt < LN_BUFF_LEN) { // if buffer not full
#if USE_INTERRUPT
      if (bNewInt[uiRfidPort]) {
        bNewInt[uiRfidPort] = false;
#else
      if (mfrc522[uiRfidPort].PICC_IsNewCardPresent()) {
#endif

        if (mfrc522[uiRfidPort].PICC_ReadCardSerial()) // if tag data
        {
          if (uiNrEmptyReads[uiRfidPort] > 2) { // send a uid only once
            //if (! compareUid( mfrc522[uiRfidPort].uid.uidByte, oldUid[uiRfidPort], mfrc522[uiRfidPort].uid.size)) // if the card-ID differs from the previous card-ID - original GCA51
            //{
            // set sensor Active/LOW (see rfid2ln)
            bitWrite(svtable.svt.pincfg[uiRfidPort].value2, 4, 0x0); // Store state in input[n].value2 bit because the next LocoNet.send (OPC_INPUT_REP.... function needs this information
            LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[uiRfidPort].value1, svtable.svt.pincfg[uiRfidPort].value2); // Send the state change to LocoNet
            Serial.println();
            Serial.print(F("Sending sensor state ACTIVE. Address: ")); Serial.println(softwareAddress[uiRfidPort], DEC);
            bSensorActive[uiRfidPort] = true;

            if (bSerialOk) { // Show some details of the PICC (that is: the tag/card)
              Serial.print(F("Port: "));
              Serial.print(uiRfidPort);
              Serial.print(F(" Card UID:"));
              dump_byte_array(mfrc522[uiRfidPort].uid.uidByte, mfrc522[uiRfidPort].uid.size);
              Serial.println();
            }

            // GCA51 v150 code updated for latest rfid2ln,
            setMessageHeader(uiBufWrIdx, uiRfidPort); // fills SendPacketSensor[i].data[0] till .data[4] with data. Address used is the port software address of uiRfidPort.

#ifdef JMRI4

            SendPacketSensor[uiBufWrIdx].data[7] = 0;                                                    // clear the byte for the MS bits of the data bytes
            // Fill SendPacketSensor.data[5-6] with 2 data bytes from the RFID card UID (becomes the Id Tag in JMRI).
            // Note: works as hi and lo bytes so must flip. Only 7 bits per LocoNet byte
            if (mfrc522[uiRfidPort].uid.size > 1)
            {
              SendPacketSensor[uiBufWrIdx].data[5] = mfrc522[uiRfidPort].uid.uidByte[1] & 0x7F;          // lowAddress. LocoNet bytes have only 7 bits;
              SendPacketSensor[uiBufWrIdx].data[6] = mfrc522[uiRfidPort].uid.uidByte[2] & 0x7F;          // highAddress. LocoNet bytes have only 7 bits;
            }
            // no room for MSB but onlu make unique
            SendPacketSensor[uiBufWrIdx].data[7] = 0xFF;                                                 // Reset the checksum data byte
            for (j = 0; j < 7; j++)
            {
              SendPacketSensor[uiBufWrIdx].data[7] ^= SendPacketSensor[uiBufWrIdx].data[j];              // calculate the checksum for header + data of the RFID-7 message
            }

#else
            SendPacketSensor[uiBufWrIdx].data[12] = 0;                                                    // clear the byte for the MS bits of the data bytes
            for (i = 0, j = 5; i < UID_LEN; i++, j++)                                                     // Fill SendPacketSensor.data[5-12] with 7 data bytes information from the RFID card
            {
              if (mfrc522[uiRfidPort].uid.size > i)
              {
                SendPacketSensor[uiBufWrIdx].data[j] = mfrc522[uiRfidPort].uid.uidByte[i] & 0x7F;         // LocoNet bytes have only 7 bits;
                // MSbit is transmitted in the SendPacket.data[10] = LnMessage[12])
                if (mfrc522[uiRfidPort].uid.uidByte[i] & 0x80)                                            // if there is an MSB-bit in the data-byte
                {
                  SendPacketSensor[uiBufWrIdx].data[12] |= 1 << i;                                        // bring this MSB-bit to the last databyte [12], together with the other MSB-bits
                }
              } else {
                SendPacketSensor[uiBufWrIdx].data[j] = 0;
              }
            } // for (uint8_t i=0, j=5

            SendPacketSensor[uiBufWrIdx].data[13] = 0xFF;                                                   // Reset the checksum data byte
            for (j = 0; j < 13; j++)
            {
              SendPacketSensor[uiBufWrIdx].data[13] ^= SendPacketSensor[uiBufWrIdx].data[j];                // calculate the checksum for header + data of the RFID-7 message
            }
#endif
#ifdef DEBUG
            if (bSerialOk) {
              Serial.print(uiRfidPort); Serial.print(F(" built LN mess.: "));
              dump_byte_array(SendPacketSensor[uiBufWrIdx].data, uiLnSendLength);
              Serial.println();
            }
#endif

            // LnSend was here in v150 >

            if (uiBufWrIdx < LN_BUFF_LEN) {
              uiBufWrIdx++;
            } else {
              uiBufWrIdx = 0;
            }
            uiBufCnt++;

            // Serial.print("uiBufWrIdx: "); Serial.print(uiBufWrIdx); Serial.print(" uiBufCnt up: "); Serial.println(uiBufCnt);

            //} // if(!compareUid( mfrc522[uiRfidPort] - from v150 code

          } // if(uiNrEmptyReads[uiRfidPort] > 2)

          uiNrEmptyReads[uiRfidPort] = 0;
        } // if(mfrc522[uiRfidPort].PICC_ReadCardSerial())

#if USE_INTERRUPT
        //clearInt(mfrc522[uiRfidPort]);
        //activateRec(mfrc522[uiRfidPort]); // rearm the reading part of the mfrc522
#endif
      } else { // if newCard / newInt
        /* Reset the sensor indication in Rocrail => RFID can be used as a normal sensor */

        if (uiNrEmptyReads[uiRfidPort] == MAX_EMPTY_READS)
        {
          if (bSensorActive[uiRfidPort])
          {
#ifdef DEBUG
            if (bSerialOk) {
              Serial.print(F("Sending sensor state INACTIVE. Address: ")); Serial.println(softwareAddress[uiRfidPort], DEC); Serial.println(); // spacer
            }
#endif
            // set sensor Inactive/HIGH (see rfid2ln)
            bitWrite(svtable.svt.pincfg[uiRfidPort].value2, 4, 0x1); // Store state in input[n].value2 bit because the next LocoNet.send (OPC_INPUT_REP... function needs this information
            LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[uiRfidPort].value1, svtable.svt.pincfg[uiRfidPort].value2); // Send the state change to LocoNet
          }

        } //if(uiNrEmptyReads

        if (uiNrEmptyReads[uiRfidPort] <= (MAX_EMPTY_READS + 1)) {
          uiNrEmptyReads[uiRfidPort]++;
        } // if ( !rc mfrc522.PICC_ReadCard
      } // else if ( mfrc522.PICC_IsNewCardPresent()

      copyUid(mfrc522[uiRfidPort].uid.uidByte, oldUid[uiRfidPort], mfrc522[uiRfidPort].uid.size);     // fill oldUid with the latest card-ID, send just once in 2 sec.
      resetUid[uiRfidPort] = millis() + 2000; // record the time. After 2 seconds the main loop() releases the blockade on the same rfid ID

      // Halt PICCs
      mfrc522[uiRfidPort].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[uiRfidPort].PCD_StopCrypto1();

      if (resetUid[uiRfidPort] < millis())
      {
        oldUid[uiRfidPort][0] = 0;                                                                // Release, after 2 seconds, the blockade on the same rfid ID for Reader[i]
        oldUid[uiRfidPort][1] = 0;                                                                // 2 zeros at start of UID never seen, so enough to pass
      }

      uiRfidPort++;
      if (uiRfidPort == uiActReaders + uiFirstReaderIdx) { // loop over only connected readers
        uiRfidPort = uiFirstReaderIdx;
      }

      // Arduino IDE signals 1 too many } caused by #if USE_INTERRUPT at head of RFID if
    } // if(uiBufCnt < LN_BUFF_LEN

  } // end of if(uiActReaders>0 RFID

  /******************** SEND **************************
     Send the tag data on the LocoNet bus
     Total message size is 14 bytes, 7 RFID UID bytes
  */
  if (uiBufCnt > 0)
  {
    LocoNet_communication(1); // turn on onboard LocoLED - blink 1x + sensor change
    LN_STATUS lnSent = LocoNet.send( &SendPacketSensor[uiBufRdIdx], LN_BACKOFF_MAX - (ucBoardAddrLo % 10)); // trying to differentiate the ln answer time
    if (lnSent = LN_DONE) // message sent OK
    {
#ifdef DEBUG
      // from JMRI 5.12ish LnInterpret() GCA51 RFID-7 size = 12 bytes
      Serial.print(F("LN mess sent successfully for RFID tag uid (decoded to verify): "));
      // add rfid_hi byte

      int start = 5;
      int end = 12;
#ifdef JMRI4
      start = 5;
      end = 7;
#endif

      char hexCar[2];
      int j, hi;
      int rfidHi = SendPacketSensor[uiBufRdIdx].data[end]; // MSbits are transmitted via byte [12]
      for (j = start; j < end; j++)                        // print the 7 RFID UID HEX bytes
      {
        hi = 0x0;
        if ((rfidHi >> j) % 2 == 1) hi = 0x80;
        sprintf(hexCar, "%02X", SendPacketSensor[uiBufRdIdx].data[j] + hi); // 02X means: pad with leading 0's if required
        Serial.print(hexCar); Serial.print(" "); // space bytes
      }
      Serial.println();
#endif

      if (uiBufRdIdx < LN_BUFF_LEN) {
        uiBufRdIdx++;
      } else {
        uiBufRdIdx = 0;
      }
      uiBufCnt--;
      // Serial.print("uiBufRdIdx: "); Serial.print(uiBufRdIdx); Serial.print(" uiBufCnt down: "); Serial.println(uiBufCnt);
    } // if(lnSent = LN_DONE)

  } // if(uiBufCnt > 0

} // end of Arduino loop()



/********************************** RFID methods ***************************************

***************************************************************************************************************************
   Purpose: calculate the adrr.low and addr.high bytes, needed for the RFID-7 report to Rocrail or other systems.
   The software address of the specific port is calculated from the CV's of this port (but we also store it in softwareAddress?!)

   rfIndex = index of rfid reader
   pIndex = index of SendPacketSensor, 0 or 1 on GCA51
***************************************************************************************************************************/
void setMessageHeader(uint8_t rfIndex, uint8_t pIndex)
{
  uint8_t odd_even;
  uint16_t sensorAddr;

  SendPacketSensor[rfIndex].data[0] = 0xE4; // OPC - variable length message OPC_LISSY_REP
  SendPacketSensor[rfIndex].data[1] = uiLnSendLength;

  if (bitRead (svtable.svt.pincfg[pIndex].value2, 5)) odd_even = 2;                                            // bitread for bit 5 on SV5, SV8, SV11, SV14 etc.
  else odd_even = 1;

  sensorAddr = (((svtable.svt.pincfg[pIndex].value2 & 0x0F) << 8 ) + (svtable.svt.pincfg[pIndex].value1 << 1 ) + odd_even); // Calculated software address of the port. For Port 1 .value1 == SV4 and .value2 == SV5
  //           (SV5 & 0x0F) << 8 == high byte                    +  SV4 << 1 == low byte                   + odd_even       == software-address of the hardware port

  // sensorAddr is the complete sensor address.
  // Now we send this Address to LocoNet but we must split the port softwareAddress into a high part and a low part
  uint8_t addrHi = sensorAddr >> 7;
  uint8_t addrLo = sensorAddr & 0x7F;

  //#ifdef DEBUG
  //  Serial.print ("sensorAddr = "); Serial.println (sensorAddr);
  //  Serial.print ("Addr_high  = "); Serial.println (sensorAddr >> 7);
  //  Serial.print ("Addr_low   = "); Serial.println (sensorAddr & 0x7F);
  //#endif

#ifdef JMRI4                                                                                              // LISSY IR report to JMRI 4.22-5.21
  SendPacketSensor[rfIndex].data[2] = 0x00; // short LISSY IR report                                      // category 1 (valid: 0x0 - 0x3)
  SendPacketSensor[rfIndex].data[3] = 0x40;                                                               // report type Loco seen. Direction North, South += 0x20
  SendPacketSensor[rfIndex].data[4] = addrLo;                                                             // only 1 address byte for reader/sensor, so max. 0x7F = LR126
  if (addrHi != 0x0) {
    Serial.print ("RFID sensorAddr. "); Serial.print (sensorAddr); Serial.print (" reduced to low byte only: "); Serial.println (addrLo);
  }
# else
  SendPacketSensor[rfIndex].data[2] = 0x41; // report type unique to RFID-5/7
  SendPacketSensor[rfIndex].data[3] = addrHi;                                                             // addr.high for RFID-7 report to Rocrail or JMRI 5.12+
  SendPacketSensor[rfIndex].data[4] = addrLo;                                                             // addr.low  for RFID-7 report
#endif

}

/********************************** LocoIO methods ***************************************
  Supported .cnfg values in GCA51 v151 see struct configOptions in header

  Inputs:
  (!bitRead(svtable.svt.pincfg[n].cnfg, 7), active Low
  -   7 Toggle Indirect
  -  15 Toggle
  -  27 Off
  -  31 Block detector (default)
  -  91 Block detector - delayed
  -  103 Push button - active Low - Indirect
  -  47 Push button - active Low

  Outputs:
  (bitRead(svtable.svt.pincfg[n].cnfg, 7)
  if bit 0 of .cnfg == 1 then output is high at startup (meaning Off on GCA51 with fixed Active Low)
  svt.pincfg[n].value2 bits 4-7 == 1: 1 ==3: 2 (for 2 alternating lights on same software address)
  - 128 Fixed 1 - Off             .value2 bits 4-7 = 1
  - 129 Fixed 1 - On              .value2 bits 4-7 = 1
  - 144 Fixed 1 - Off - Blink     .value2 bits 4-7 = 1
  - 145 Fixed 1 - On - Blink      .value2 bits 4-7 = 1

  - 128 Fixed 2 - Off             .value2 bits 4-7 = 3
  - 129 Fixed 2 - On              .value2 bits 4-7 = 3
  - 144 Fixed 2 - Off - Blink     .value2 bits 4-7 = 3
  - 145 Fixed 2 - On - Blink      .value2 bits 4-7 = 3
  - 192 Block detector
  - 208 Block detector - Blink

  Other LocoIO functions common with GCA50a are in CGA51Func.cpp */

boolean processPeerPacket()
{
  // Check is a OPC_PEER_XFER message
  if (LnPacket->px.command != OPC_PEER_XFER) return (false);

  //Check is my destination
  if ((LnPacket->px.dst_l != 0 || LnPacket->px.d5 != 0) &&
      (LnPacket->px.dst_l != 0x7f || LnPacket->px.d5 != svtable.svt.addr_high) &&
      (LnPacket->px.dst_l != svtable.svt.addr_low || LnPacket->px.d5 != svtable.svt.addr_high))
  {
#ifdef DEBUG
    Serial.println("OPC_PEER_XFER not for me!");
    Serial.print("LnPacket->px.dst_l: "); Serial.print(LnPacket->px.dst_l); Serial.print(" Addr low: "); Serial.println(svtable.svt.addr_low);
    Serial.print("LnPacket->px.d5: "); Serial.print(LnPacket->px.d5); Serial.print(" Addr high: "); Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.dst_h: "); Serial.print(LnPacket->px.dst_h); Serial.print(" Addr high: "); Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.d1: "); Serial.println(LnPacket->px.d1);
    Serial.print("LnPacket->px.d2: "); Serial.println(LnPacket->px.d2);
#endif
    return (false);
  }

  //Set high bits in correct position
  bitWrite(LnPacket->px.d1, 7, bitRead(LnPacket->px.pxct1, 0));
  bitWrite(LnPacket->px.d2, 7, bitRead(LnPacket->px.pxct1, 1));
  bitWrite(LnPacket->px.d3, 7, bitRead(LnPacket->px.pxct1, 2));
  bitWrite(LnPacket->px.d4, 7, bitRead(LnPacket->px.pxct1, 3));

  bitWrite(LnPacket->px.d5, 7, bitRead(LnPacket->px.pxct2, 0));
  bitWrite(LnPacket->px.d6, 7, bitRead(LnPacket->px.pxct2, 1));
  bitWrite(LnPacket->px.d7, 7, bitRead(LnPacket->px.pxct2, 2));
  bitWrite(LnPacket->px.d8, 7, bitRead(LnPacket->px.pxct2, 3));

  //OPC_PEER_XFER D1 -> Command (1 =SV write, 2 = SV read)
  //OPC_PEER_XFER D2 -> Register to read or write
  if (LnPacket->px.d1 == 2)
  {
#ifdef DEBUG
    Serial.print("READ "); Serial.print(LnPacket->px.d2); Serial.print(" "); Serial.print(LnPacket->px.d2 + 1); Serial.print(" "); Serial.println(LnPacket->px.d2 + 2);
#endif
    sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2 + 1], svtable.data[LnPacket->px.d2 + 2]);
    return (true);
  }

  // Write command
  if (LnPacket->px.d1 == 1)
  {

    if (LnPacket->px.d2 > 0) // SV 0 contains the program version - TODO fix to SV100
    {
      //Store data
      svtable.data[LnPacket->px.d2] = LnPacket->px.d4;
      EEPROM.write(LnPacket->px.d2, LnPacket->px.d4);

#ifdef DEBUG
      Serial.print("WRITE "); Serial.print(LnPacket->px.d2); Serial.print(" <== ");
      Serial.print(LnPacket->px.d4); Serial.print(" | ");
      Serial.print(LnPacket->px.d4, HEX); Serial.print(" | ");
      Serial.println(LnPacket->px.d4, BIN);
#endif
    }

    // Answer packet
    LocoNet_communication(1); // turn on LocoLED
    sendPeerPacket(0x00, 0x00, LnPacket->px.d4);
#ifdef DEBUG
    Serial.println(">> OPC_PEER_XFER reply sent");
#endif
    return (true);
  }

  return (false);
}

void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2)
{
  lnMsg txPacket;

  txPacket.px.command = OPC_PEER_XFER;
  txPacket.px.mesg_size = 0x10;
  txPacket.px.src = svtable.svt.addr_low;
  txPacket.px.dst_l = LnPacket->px.src;
  txPacket.px.dst_h = LnPacket->px.dst_h;
  txPacket.px.pxct1 = 0x00;
  txPacket.px.d1 = LnPacket->px.d1;                       // Original command
  txPacket.px.d2 = LnPacket->px.d2;                       // SV requested
  txPacket.px.d3 = svtable.svt.vrsion;
  txPacket.px.d4 = 0x00;
  txPacket.px.pxct2 = 0x00;
  txPacket.px.d5 = svtable.svt.addr_high;                 // SOURCE high address
  txPacket.px.d6 = p0;
  txPacket.px.d7 = p1;
  txPacket.px.d8 = p2;

  //Set high bits in correct position
  bitWrite(txPacket.px.pxct1, 0, bitRead(txPacket.px.d1, 7));
  bitClear(txPacket.px.d1, 7);
  bitWrite(txPacket.px.pxct1, 1, bitRead(txPacket.px.d2, 7));
  bitClear(txPacket.px.d2, 7);
  bitWrite(txPacket.px.pxct1, 2, bitRead(txPacket.px.d3, 7));
  bitClear(txPacket.px.d3, 7);
  bitWrite(txPacket.px.pxct1, 3, bitRead(txPacket.px.d4, 7));
  bitClear(txPacket.px.d4, 7);
  bitWrite(txPacket.px.pxct2, 0, bitRead(txPacket.px.d5, 7));
  bitClear(txPacket.px.d5, 7);
  bitWrite(txPacket.px.pxct2, 1, bitRead(txPacket.px.d6, 7));
  bitClear(txPacket.px.d6, 7);
  bitWrite(txPacket.px.pxct2, 2, bitRead(txPacket.px.d7, 7));
  bitClear(txPacket.px.d7, 7);
  bitWrite(txPacket.px.pxct2, 3, bitRead(txPacket.px.d8, 7));
  bitClear(txPacket.px.d8, 7);

  LocoNet_communication(1); // turn on LocoLED
  LocoNet.send(&txPacket);

#ifdef DEBUG
  Serial.println("Packet sent!");
#endif
}


/*********************************************************************************************************************
  Purpose:
  Description : This call-back function is called from LocoNet.processSwitchSensorMessage
                for all Switch Request messages
                In the LocoNet.processSwitchSensorMessage is a pointer to this function
                The pointer is actually the name of this function
                This function always sends two commands within 0.5 seconds.
                In both commands the Address and Direction are the same, only Output switches from 1 to 0 within this 0.5 second
                This behavior prevents the turnout coil becoming too hot.
                In this function we only deal with steady state outputs, so we only use the variables Direction and Address

                Adapted from GCA51 v150 LocoIO (n=8; n<16) and extra bits checked.
                TODO add support for config Inverted, Nr 1/Nr 2 pairs?
**********************************************************************************************************************/
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int n;

  Direction ? Direction = 1 : Direction = 0; // Direction must be changed to 0 or 1, not 0 or 32

  LocoNet_communication(1); // turn on LocoLED

#ifdef DEBUG
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
#endif

  // Check if Address is assigned on this board, configured as output and same Direction
  for (n = 8; n < 16; n++)
  {
    if ((softwareAddress[n] == Address) && (bitRead(svtable.svt.pincfg[n].cnfg, 7))) // Set up as an Output
    {
#ifdef DEBUG
      Serial.print("Output assigned to port ");
      Serial.print(n + 1); Serial.print(" and pin "); Serial.println(pinMap[n]);
#endif
      // If pulse (hardware reset) and Direction, only listen for ON message
      if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 1 && bitRead(svtable.svt.pincfg[n].value2, 5) == Direction && Output)
      {
        digitalWrite(pinMap[n], HIGH);
        delay(PulseTime);  // pulse delay is board configured: The length of the pulse is then between 1 and 2 blink pulse length
        digitalWrite(pinMap[n], LOW);
        break;
      }
      // If continue and hardware reset and Direction
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 0 && bitRead(svtable.svt.pincfg[n].cnfg, 2) == 1 && bitRead(svtable.svt.pincfg[n].value2, 5) == Direction)
      {
        Serial.println("Hardware reset output reset"); // pulse duration?
        if (Output)
          digitalWrite(pinMap[n], HIGH); // turn off
        else
          digitalWrite(pinMap[n], LOW);
        break;
      }
      // If continue and software reset, one Direction ON turns on and other Direction ON turns off
      // OFF messages are not listened for
      else if (bitRead(svtable.svt.pincfg[n].cnfg, 3) == 0 && bitRead(svtable.svt.pincfg[n].cnfg, 2) == 0 && Output)
      {
        if (!Direction)
          digitalWrite(pinMap[n], HIGH);
        else
          digitalWrite(pinMap[n], LOW);
        break;
      }
    }
  }
}
/*********************************************************************************************************************
  GCA51 v150 variant, simpler
**********************************************************************************************************************/
//void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
//{
//  int n, i, temp_value1;
//
//   LocoNet_communication(1);                                                                         // turn on LocoLED
//
//  if (Direction > 0) Direction=1;  else Direction = 0;                                               // Direction must be changed to 0 or 1, not 0 or 32
//
//  for (n=8; n<16; n++)                                                                               // Check if Address is assigned on this board, configured as output and same Direction
//  {
//    if ((softwareAddress[n]  == Address) &&  (bitRead(svtable.svt.pincfg[n].cnfg,7) == 1))           // Address OK and setup as Output?
//        {
//          #ifdef DEBUG
//          Serial.print ("found port address: "); Serial.println(Address, DEC);
//          #endif
//          break;                                                                                     // found the hardware port
//        }
//  }
//
//    if (Direction) digitalWrite(pinMap[n-8], HIGH);        // array softwareAddress[16] and pinMap[8] are working together. softwareAddress[16] contains the software addresses of all 16 ports
//    else           digitalWrite(pinMap[n-8], LOW);         // pinMap[16] has all the 16 hardware pin numbers
//}


/*********************************************************************************************************************
  Purpose:      Handle blink timer if output commanded state is ON. Turn output off if commanded state is OFF.
  Description : Adapted from GCA51 v150 LocoIO (n=8; n<16) and .cfg bit 4 checked.

  Globals:
  blinkRate (Board setting) is in range 0 - 15
  blinkPeriod is in range 2000 ms (@0) - 250 ms (@15)
  blinkDuration = blinkPeriod/2

  Blink config bit 4:
  0d144 = 0b10010000
  0d145 = 0b10010001
  0d208 = 0b11010000
  No blink:
  0d128 = 0b10000000
  0d129 = 0b10000001
**********************************************************************************************************************/
void updateBlink(uint8_t portIdx) {

  if (bitRead(svtable.svt.pincfg[portIdx].cnfg, 4)) // // only blink for cnfg 144/145/208
  {
    if (bitRead(svtable.svt.pincfg[portIdx].value2, 4) == HIGH) // only blink when output is ON
    {
      if (blinkState[portIdx] == 1)
      {
        // if output is currently off, wait for the interval to expire before turning it on
        if (currentBlinkMillis - previousBlinkMillis >= blinkDuration)
        { // time is up, so change the state to LOW (on)
          digitalWrite(pinMap[portIdx - 8], LOW);
          blinkState[portIdx] = 0;
          previousBlinkMillis = previousBlinkMillis + blinkDuration; // save the time when we changed to on
        }
      }
      else if (blinkState[portIdx] == 0)
      {
        // if output is currently on, wait for the duration to expire before turning it off
        if (currentBlinkMillis - previousBlinkMillis >= blinkDuration)
        { // time is up, so change the state to HIGH (off)
          digitalWrite(pinMap[portIdx - 8], HIGH);
          blinkState[portIdx] = 1;
          previousBlinkMillis = previousBlinkMillis + blinkDuration; // save the time when we changed to off
        }
      } else {
        Serial.print("Unexpected updateBlink() for port "); Serial.println(portIdx);
      }
    } else { // output commanded state if HIGH (off) so turn off pin
      digitalWrite(pinMap[portIdx - 8], HIGH);
      blinkState[portIdx] == 0;
      Serial.print("Turned off output for port "); Serial.println(portIdx);
    }
  }
}
