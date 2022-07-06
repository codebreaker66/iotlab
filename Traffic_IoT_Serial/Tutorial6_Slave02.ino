//------------------------------------------------------------------------------------------------------------------
/**
***********************************************************
  IoT Future LAB - Tutorial 6 - vernetze Ampelwelt
                   Slave mit Adresse 02 - Version A
  Ver. 1.0
  Date   :  28.03.2022
  Author :  Andreas Dasbach
***********************************************************
  Ampelschaltung mit Seriellem TrafficIO-Interface
  zur Kommunkation mehrerer Ampeln untereinander
*                                                         *
*                                                         *
  Lerninhalte:
   - Kommunikation von Ampeln über Protokolle
***********************************************************
  Aenderungen
  Datum    Change
*                                                         *
***********************************************************
*/
#include <TrafficIoTSerial.h>

// Definition der Kommandos
#define FUSSGAENGERANFORDERUNG   01
#define HAUPTRICHTUNGGRUEN       02
#define HAUPTRICHTUNGROT         03

/* ******* Datenprotokoll fuer die Ampelnkommunikation ****
   Block   Byte    Beschreibung                                   *
   Header  [0]     Startbyte 1: 0xEF
           [1]     Startbyte 2: 0xFA
           [2]     Empfaengeradresse (bei 0x01 = Broadcast)
           [3]     Senderadresse     (bei 0x01 = Master)
           [4]     Datenlaenge (ohne 5 Byte Haeder)

   Daten   [0]     Eigener Status
           [1]     Kommando - siehe defines
           [2]     Folgekommando
           [3]     Zeit in Sekunden bis Folgekommando

           [4]     CRC-8 Checksumme ueber Header + Daten
*/


// Variablen definieren
TrafficIoTSerial      iotSerial;
SendData              sendData = {0x00, 0x00, 0x00, 0x00, 0x00};
SendData              recvData;
uint8_t               sendLen = 5;
uint8_t               recvLen;
uint8_t               ownAdress = 0x02; Damit ist es ei SLAVE (2)
uint16_t              lcount = 0;

// Array zur Definition der Ausgaenge
int ledPins[6] = {2, 3, 4, 5, 6, 7};

// Array zur Definition der Ampelphasen
//{R, Y, G, R, Y, G} -> Hauptrichtung / Querstrasse
int ampelphasen[8][6] = {
  {0, 0, 1, 1, 0, 0},  // Phase 0 Hauptrichtung Gruen
  {0, 1, 0, 1, 0, 0},  // Phase 1
  {1, 0, 0, 1, 0, 0},  // Phase 2
  {1, 0, 0, 1, 1, 0},  // Phase 3
  {1, 0, 0, 0, 0, 1},  // Phase 4 Querstrasse Gruen
  {1, 0, 0, 0, 1, 0},  // Phase 5
  {1, 0, 0, 1, 0, 0},  // Phase 6
  {1, 1, 0, 1, 0, 0}   // Phase 7
};

// Array mit den Wartezeiten zwischen den Ampelphasen
int wartezeiten[8] = {7000, 1000, 500, 1000, 5000, 1000, 500, 1000};

// Einfache Varbiablen
int             myState = 0;      // Speichert die aktuelle Ampelphase
int             myLastState = 0;  // Speichert die letzte Ampelphase
unsigned long   myLastTime;       // Speichert die letzte Zeitmarke
unsigned long   myAddTime = 0;    // Wartezeit aus IoT-Interface
boolean         myAnforderung = false;

// Variablen fuer den Opto-Reflexkoppler CNY70
int             mySensorPin = A0;
int             mySensorValue = 0;
// Einstellung der Schwelle fuer die Ausloesung
int             mySensorThreshold = 100;

// ***********************************************************
// * Setup Funktion                                          *
// * Wird nach dem Start/Reset einmal durchlaufen            *
// * Hier werden Initialisierungen durchgefuehrt             *
// ***********************************************************
void setup() {
  // Alle im Array definierten Ausgaunge werden als Output
  // definiert. Zugriff auf das Array erfolgt mit einer
  // for-Schleife
  for (int i = 0; i < 6; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Der Serielle Monitor wird mit seiner definierten
  // Uebertragungsgeschwindigkeit gestartet
  Serial.begin(115200);

  iotSerial.begin(ownAdress, false);
 
  // Erste Ampelphase setzen
  zeigeAmpelphase();
}

// ***********************************************************
// * Loop Funktion                                           *
// * Wird nach der Setup Funktion immmer wieder und wieder   *
// * durchlaufen. Hier fuehren wir die Dinge durch, die wir  *
// * mit unserem Programm erreichen wollen                   *
// ***********************************************************
void loop() {
  uint8_t c;

  // IoT-Interface auf empfangene Daten abfragen
  // Wenn Daten emfangen wurden gibt recvLen die
  // Anzahl der Empfangenen Zeichen wieder
  recvLen = iotSerial.receive(recvData);

  // Wir haben Zeichen empfangem
  if (recvLen > 0) {
    Serial.println("IoT-Interface Receive: *****************");
    iotSerial.printBuffer(false);

    // Daten auswerten....
    if (recvData[1] == HAUPTRICHTUNGROT) {
      // OK: Hauptrichtung soll auf Rot gehen
      // Aber eventuell mit verzoegerter Ausloesung
      myAnforderung = true;
      // eventuell gibt es eine zusätzliche Wartezeit
      myAddTime = millis() + recvData[3] * 1000;
    }
    Serial.println("****************************************");
  }

  if (myLastTime + wartezeiten[myState] < millis()
      && myAddTime < millis()
      && ((myAnforderung) || myState != 0)) {

    // Die aktuelle Zeit merken
    myLastTime = millis();
  
    // Wechsel in den naechsten Ampelstatus
    myState++;

    // Status versenden
    // per Broadcast Adress 0x01
    //sendData[7] = myState;
    //iotSerial.send(sendData, 8, 0x01);

    // Die Anforderung ist abgearbeitet...
    if (myState == 6) {
      myAnforderung = false;
    }

    // Letzter Status erreicht? zurueck zum Anfang.....
    if (myState >= 8) {
      myState = 0;
    }

    // Ausgabe einer Meldung im seriellen Monitor
    // WICHTIG: die Funktion print erzeugt keine neue Zeile!!!
    //Serial.print("Naechste Ampelphase ");
    //Serial.print(myState);
    //Serial.print(" - ");
    //Serial.print("Wartezeit ");
    //Serial.print(wartezeiten[myState]);
    //Serial.println(" ms");    // println erzeugt eine neue Zeile!!!!
  }

  // Hat sich der Status geaendert?
  if (myState != myLastState) {
    // OK - es gibt etwas zu tun ....

    // zuerts den aktuellen Status merken.
    myLastState = myState;

    // Durch Aufruf der Funktion zeigen alle Ampeln den aktuellen
    // Status.
    zeigeAmpelphase();
  }

  //  // Analogen Eingang auslesen wenn aktuell keine Anforderung
  //  // abgearbeitet wird
  //  mySensorValue = analogRead(mySensorPin);
  //  // Ausgabe fuer den Seriallen Plotter....
  //  //Serial.println(mySensorValue);
  //  // Fuer den Seriellen Plotter nur alle 100ms den Sensor abfragen.
  //  // Dafuer nutzen wir hier mal den Delay....
  //  //delay(100);
  //
  //  if ((mySensorValue > mySensorThreshold ) && (!myAnforderung)) {
  //    //Serial.println("Anforderung Querstrasse");
  //    myAnforderung = true;
  //  }

}

// **************************************************************
// **************************************************************
// Eigene weitere Funktionen

// ***********************************************************
// * zeigeAmpelphase                                         *
// * Steuert alle Ausgaenge des Arrays an und uebergibt den  *
// * aktuellen Status.                               *
// ***********************************************************
void zeigeAmpelphase() {

  // For-Schleife ueber alle Ausgaenge
  for (int i = 0; i < 6; i++) {
    //Setzen des aktuellen Ausgangs zur aktuellen Phase
    digitalWrite(ledPins[i], ampelphasen[myState][i]);
  }

  // Ausgabe an den seriellen Monitor
  //Serial.print("Setze die Ampeln für den Status ");
  //Serial.println(myState); // println erzeugt eine neue Zeile!!!!
  //Serial.println("*-------------------------------------------*");
}
