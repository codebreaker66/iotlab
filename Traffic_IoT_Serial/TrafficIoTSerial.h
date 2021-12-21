/*
  TrafficIoTSerial.h - Library for IoT Traffic-Light communication
  Copyright (c) 2020 Andreas Dasbach.  All right reserved.
*/

// ensure this library description is only included once
#ifndef TrafficIoTSerial_h
#define TrafficIoTSerial_h

#include "Arduino.h"
#include "AltSoftSerial.h"

typedef char SendData[20];
//typedef uint8_t SendData[20];

// library interface description
class TrafficIoTSerial
{
  // user-accessible "public" interface
public:
	TrafficIoTSerial();
	boolean begin(char myaddress, boolean debug);
	int send(char * data, char len, char address);
	uint8_t receive(char * data);
	boolean isBusy();
	void    printBuffer(boolean send);
	int     read();

  // library-accessible "private" interface
private:
	//Membervariables
	AltSoftSerial                 _altSerial;
	boolean             _debug = false;     // Schaltet die Printausgabe an
	signed long             _next;                            // Zur Erzeugung eines 1ms Taktes
	char                    _myAddress;                 // Adresse des Busteilnehmers
	boolean             _busy;              // Bei true kein versenden moeglich
	uint8_t             _timeout = 100;     // TimeOut fuer Receive
	uint8_t                                _recvCounter = 0;   // Counter auf die Daten
	boolean             _bypass = false;    // Uebertragung wird durchgeleitet
	boolean             _broadcast = false; // Uebertragunf ist eine Broascastmeldung an alle
	boolean             _init = false;      // Ist das Interface initialisiert
	char                                 _sendBuffer[26];    // Puffer fuer die fertige Nachricht
	char                                 _recvBuffer[26];    // Empfangspuffer fuer die Nachricht
	uint8_t             _CRCStart = 0xFF;   // STartwert fuer die CRC-Berechnung
	unsigned long       _XorOp;
	uint8_t             _checksum = 0;
	// Private Methods
	void logMsg(String msg);
};

#endif
