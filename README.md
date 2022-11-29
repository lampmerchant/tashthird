# TashThird

Emulator for Global Village TelePort A300 ADB modem.

This firmware for the PIC12F1840 appears as a Global Village TelePort A300 ADB modem and links it to the PIC's UART with RTS/CTS control lines.  The pushbutton cycles between four modes: echo, 300 baud, 1200 baud, and 2400 baud.  The echo mode is the default and seems to be necessary to allow ZTerm to initialize.  The currently selected baud rate is displayed in the TelePort menubar display.
