/*
  TrafficIoTSerial.h - Library for IoT Traffic-Light communication
  Copyright (c) 2020 Andreas Dasbach.  All right reserved.
*/

// include this library's description file
#include "TrafficIoTSerial.h"
#include "AltSoftSerial.h"


/*
AltSoftSerial mySerial;
Create the AltSoftSerial object. Only one AltSoftSerial can be used, with the fixed pin assignments shown above.

mySerial.begin(baud);
Initialize the port to communicate at a specific baud rate.

mySerial.print(anything);
Print a number or text. This works the same as Serial.print().

mySerial.available();
Returns the number of bytes received, which can be read.

mySerial.read();
Reads the next byte from the port. If nothing has been received, -1 is returned.
*/

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of iItnstances
TrafficIoTSerial::TrafficIoTSerial()
{
  // initialize this instance's variables

}

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12

/* Todo:
 - Mit Token arbeiten... Senden nur wenn leeres Token empfangen.

*/

// Public Methods //////////////////////////////////////////////////////////////
/**
 * Initialize the TrafficIotInterface for starting communication
 *
 * address 0x00 is the address for the master
 *
 * @param myaddress is a character with the own address
 * @param debug is a boolean for activate debug output on hardware serial.
 * @return true when init was successful
 */
boolean TrafficIoTSerial::begin(char myaddress, boolean debug)
{
	// Pruefen ob die Adresse korrekt
	if (myaddress < 1 or myaddress > 0x7F) {
			return false;
	}

	_myAddress = myaddress;
	_debug = debug;
	_altSerial.begin(9600);
	_init = true;
}

/**
 * Call send for sending data to a receiver. returns true until
 * data sending is finished
 *
 * address 0x01 is the broadcast address to all receiver
 *
 * @param data is a character pointer to the data for send
 * @param len is a character with the len of the data.
 * @param address is a character with the address of the receiver
 * @return the number of bytes waiting for sending
 */
int TrafficIoTSerial::send(char * data, char len, char address)
{
	// Senden ... wenn nicht beschaeftigt!
	if (!_busy) {
		logMsg("Start Sending");
		_busy = true;
		// Es sollen Daten versendet werden
		// Pruefen ob die Laenge gueltig ist...
		if (len < 0 or len > 20) {
				logMsg("Length-ERR");
				return -1;
		}
		// Pruefen ob die Adresse korrekt
		if (address < 0x01 or address > 0x7F) {
				logMsg("Addr-ERR");
				return -2;
		}
		// Pruefen ob Sender = Empfaenger
		if (address != 0x01 and address == _myAddress) {
				logMsg("Same Addr-ERR");
				return -3;
		}
		// Broadcast nur als Master moeglich
		/*if (address == 0x01 and _myAddress != 0x01) {
				logMsg("Master-ERR");
				return false;
		}*/

		// Jetzt versenden wir die Daten...
		// Aufbau:
		// Byte 0: Startbyte 1
		// Byte 1: Startbyte 2
		// Byte 2: Empfaengeradresse
		// Byte 3: Senderadresse
		// Byte 4: Datenlaenge
		// Byte 5 - 5+len: Daten
		_sendBuffer[0] = 0xEF;
		_sendBuffer[1] = 0xFA;
		_sendBuffer[2] = address;
		_sendBuffer[3] = _myAddress;
		_sendBuffer[4] = len;  // Laenge der eigentlichen Daten
		// ...und die Daten
		for (int i = 0; i < len; i++) {
				_sendBuffer[5+i] = data[i];
		}

		// Jetzt kann die fertige Nachricht verendet werden
		// zus. wird CRC berechnet
		_sendBuffer[len + 5] = _CRCStart;      // Startwert der CRC...

		for (int i = 0; i < len + 5; i++) {
				_altSerial.print(_sendBuffer[i]);
				//CRC berechnen
				_sendBuffer[len + 5] = _sendBuffer[len + 5] ^ _sendBuffer[i];
		}
		// und als letztes Zeichen die CRC in die Schnittstelle
		_altSerial.print(_sendBuffer[len + 5]);

		_busy = false;
		return 0;   // das Senden war erfolgreich!
	}
	return len;  // fals wir beschaeftigt sind.....
}
/**
 * Call receive every cycle. Returns true when valid in the receive buffer
 *
 * Data is valid for the own address or for broadcast address 0x00
 * Data for other receiver is send directly forward.
 *
 * @param data is a character pointer to the data received
 * @param len is a character with the len of the data received.
 * @return the number of data bytes received
 */
uint8_t TrafficIoTSerial::receive(char * data)
{
	uint8_t lcurrData;

	// aktuelle Uebertragung auf TimeOut pruefen
	if (_busy && ((signed long)(millis() - _next)) > _timeout) {
		logMsg("TimeOut");
	_busy = false;  // Ende der Uebertraung Erzwingen.
	 }

	// Start einer neuen Uebertragung erkennen
	if (_altSerial.available() && !_busy) {
		// Das erste Zeichen lesen
		lcurrData = _altSerial.read();
		// Das erste Zeichen muss ein 0xEF sein
		if (lcurrData != 0xEF) {
				logMsg("No Start1");
				return 0;
		}
		_recvCounter = 0;              // Start eine neuen Uebertragung
		_next = millis();                 // die Startzeit merken
		_busy = true;
		_broadcast = false;
		_bypass = false;
		// Zeichen einlagern
		_recvBuffer[_recvCounter] = lcurrData;
	}
	// Verarbeitung des Hauptteils der Nachricht
	else if (_altSerial.available() && _busy) {
		// Zeichen einlesen
		lcurrData = _altSerial.read();
		_recvCounter++;
		_recvBuffer[_recvCounter] = lcurrData;

		// Je nach Counter noch weitere Pruefungen....
		// 2. Zeichen ein 0xFA?
		if (_recvCounter == 1 && lcurrData != 0xFA) {
				logMsg("No Start2");
				_busy = false;
				return 0;
		}
		// Broadcast oder Bypass?
		if (_recvCounter == 3) {
				if (lcurrData == 0x01 && (uint8_t)_recvBuffer[2] == 0x01) {
					_broadcast = true;
				} else if ((uint8_t)_recvBuffer[2] != _myAddress) {
					_bypass = true;
				}
		}
		// Laenge gueltig?
		if (_recvCounter == 4 && lcurrData > 20) {
				logMsg("To long");
				_busy = false;
				return 0;
		}
		// Ist das Ende der Daten erreicht? Inkl CRC!!
		if (_recvCounter == 5 + _recvBuffer[4]) {
			// Die CRC ueber die Daten berechnen
			_recvBuffer[_recvBuffer[4] + 5] = _CRCStart;
			for (int i = 0; i < _recvBuffer[4] + 5; i++) {
				// Die netto Daten kopieren: ohne Header und CRC!
				if (i > 4) {
						 data[i-5] = _recvBuffer[i];
				}
				//CRC berechnen
				_recvBuffer[_recvBuffer[4] + 5] = _recvBuffer[_recvBuffer[4] + 5] ^ _recvBuffer[i];
			}
			// Jetzt vergleichen wir die berechnete mit der empfangenen CRC
			if ((uint8_t)_recvBuffer[_recvBuffer[4] + 5] != lcurrData) {
				logMsg("CRC Fail");
				//printBuffer(false);
				_busy = false;
				return 0;
			} else {
				//********************************************************************************
				// Wir haben eine gueltige Nachricht empfangen....
				//********************************************************************************
				logMsg("Valid Package recv.");
				
				// Selbst der Sender? Broadcast?
				if (_recvBuffer[3] == _myAddress) {
					if(_broadcast) {
						logMsg("BrCast End");
						_busy = false; // Ende, Interface freigeben
						return 0;
					} else {
						logMsg("Bad Cycle");
						_busy = false; // Ende, Interface freigeben
						return 0;
					}
				}

				// Bypass: Direkt weitersenden
				if (_bypass) {
					logMsg("Data Bypass");
					for (int i = 0; i < _recvBuffer[4] + 6; i++) {
						_altSerial.print(_sendBuffer[i]);
					}
					_busy = false; // Ende, Interface freigeben
					return 0;
				}
				// Broadcast? ...zurueck im Master?
				if (_broadcast) {
					// Weitersenden....
					logMsg("BrCast Forward");
					for (int i = 0; i < _recvBuffer[4] + 6; i++) {
						_altSerial.print(_recvBuffer[i]);
					}
					//_busy = false; // Ende, Interface freigeben
					//return (uint8_t)_recvBuffer[4];   // Empfang melden!
				}
				// Normaler Empfang ... zurueck melden!
				logMsg("Data Recv");
				_busy = false;
				return (uint8_t)_recvBuffer[4];
			}
		}
	}
	return 0;
}
/**
 * Call isBusy befor send to check if interface is busy
 *
 *
 * @return true when interface is busy
 */
boolean TrafficIoTSerial::isBusy()
{
    return _busy;
}

/**
 * Print a Log Message in the Hardware Serial if debug is true
 *
 *
 * @return void
 */
void TrafficIoTSerial::logMsg(String msg)
{
	if (_debug) {
		Serial.println(msg);
	}
}

/**
 * Print the Send/Receive buffer in the Hardware Serial if debug is true
 *
 *
 * @return void
 */
void TrafficIoTSerial::printBuffer(boolean send)
{
	uint8_t  temp;
	uint8_t  temp2;
	char*    lBuffer;

	Serial.println();
	// Sende oder Empfangsrichtung
	if (send) {
		lBuffer = _sendBuffer;
		Serial.println("Send...");
	} else {
		lBuffer = _recvBuffer;
		Serial.println("Recv...");
	}
	// Formatierte Ausgabe
	Serial.print("Start:  ");
	temp = lBuffer[0];
	Serial.print(temp, HEX);
	Serial.print(":");
	temp = lBuffer[1];
	Serial.println(temp, HEX);
	Serial.print("Empf.:  ");
	temp = lBuffer[2];
	temp2 = lBuffer[3];
	if (temp == 0x01 && temp2 == 0x01) {
		Serial.print(temp, HEX);
		Serial.println(" BrCast!");
	} else {
		Serial.println(temp, HEX);
	}
	Serial.print("Sender: ");
	Serial.println(temp2, HEX);
	Serial.print("Laenge: ");
	temp = lBuffer[4];
	Serial.println(temp, HEX);
	Serial.print("Data:   ");
	for (int i = 0; i < lBuffer[4]; i++) {
		temp = lBuffer[5+i];
		Serial.print(temp, HEX);
		if (i < lBuffer[4]-1) {
			Serial.print(":");
		}
	}
	Serial.println();
	Serial.print("CRC:    ");
	temp = lBuffer[5+lBuffer[4]];
	Serial.println(temp, HEX);
}

/**
 * A brief history of JavaDoc-style (C-style) comments.
 *
 * This is the typical JavaDoc-style C-style comment. It starts with two
 * asterisks.
 *
 * @param theory Even if there is only one possible unified theory. it is just a
 *               set of rules and equations.
 */
int TrafficIoTSerial::read()
{
        if (_altSerial.available()) {
                return _altSerial.read();
        }
        return 0;
}
