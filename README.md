![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/compile-sketches.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/sync-labels.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/code-formatting-check.yml/badge.svg)
![workflow status](https://github.com/silverailscolo/GCA51/actions/workflows/spell-check.yml/badge.svg)

## Overview
**GCA51** is a sketch to attach 2 RC522 RFID readers to an Arduino Nano based LocoIO module.
Earlier version were successfully used in RocRail and JMRI.

Beta version 151b includes the updated rfid2ln library, flexible use of 0-2 RC522 readers and startup reporting in the Serial Console and blinking outputs using the Blink Rate board setting.

_NOTE: This code is under development as per February 2025 (see Issues)._

The code requires GCA51 hardware, available as a PCB or kit from [P. Giling](https://wiki.rocrail.net/doku.php?id=gca51-en).

## Installation

Follow the Arduino IDE guidelines.

Download the mrrwa [LocoNet library](https://github.com/mrrwa/LocoNet/blob/master/LocoNet.h) as a .ZIP and install it in the IDE using the Sketch > Include Library > Add .ZIP Library... menu.

Also download the [rfid2ln](https://github.com/lmmeng/rfid2ln) library by Immeng. This sketch replaces the included .ino file, but you do need to copy both the rfid2ln.h and the rfid2lnFunc.cpp files to your Arduino IDE project folder.
