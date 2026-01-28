![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/compile-sketches.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/sync-labels.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/code-formatting-check.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/spell-check.yml/badge.svg)

## Overview
**GCA51** is a sketch to attach two RC522 RFID readers to an Arduino Nano based LocoIO module.
Earlier version were successfully used in RocRail and JMRI.

Version 1.52 includes:
- the updated rfid2ln library
- flexible use of 0 to 2 RC522 readers
- startup reporting in the Serial Console
- blinking outputs using the Blink Rate board setting
- read/configure ports using commands over serial (type H in Serial Monitor)

The code requires GCA51 hardware, available as a PCB or kit from [P. Giling](https://wiki.rocrail.net/doku.php?id=gca51-en).

### Credits
* Based on [MRRwA LocoNet libraries for Arduino](http://mrrwa.org/) and
  its LocoNet Monitor example.
* The included `rfid2ln` lib was adapted from https://github.com/lmmeng/rfid2ln
  to compile in Arduino IDE.
* Inspired on the GCA50 board from [Peter Giling](http://www.phgiling.net/)
* Also inspired by the LocoShield from [SPCoast](http://www.scuba.net/)
* Thanks also to [Rocrail group](http://www.rocrail.org)
* Thanks to the LocoNet part of the code from [Dani Guisado/ClubNCaldes](http://www.clubncaldes.com)

## Installation

Follow the Arduino IDE guidelines. 

- Install the RFID reader library "MFRC522" from the Arduino IDE > Tools > Library manager.
- Install the "SerialCommand_Advanced" library by argandas from the Arduino IDE > Tools > Library manager.

- Download the mrrwa [LocoNet library](https://github.com/mrrwa/LocoNet/blob/master/LocoNet.h) as a .ZIP and install it in the IDE using the Sketch > Include Library > Add .ZIP Library... menu.

Updated versions of the [rfid2ln](https://github.com/lmmeng/rfid2ln) library by Immeng are included in the project to compile in current Arduino IDEs.

> ### Tip:
> When uploading the sketch to the Nano in Arduino IDE fails, try Tools > Processor: ATmega328P (Old Bootloader).
