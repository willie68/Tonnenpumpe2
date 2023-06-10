/*
   Diese kleine Programm dient dazu eine Wassertonne mit Vorfilteranlage zu
   steuern. Folgende Funktionen übernimmt das Programm.

   - Wenn Vorfilter voll und Hauptspeicher nicht voll, starten einer Wasserpumpe
   mit Nachlaufzeit.
   - Pumpe wird direkt ausgeschaltet wenn Hauptspeicher voll.
   - Watchdog falls System in einem undefinerten Zustand gerät.

   Historie
   WKLA 13.07.2018
   - Watchdog implementiert
   - verschiedene Zeitkonstanten für Debug und nicht Debug version
   - Board LED als Status LED

   WKLA 16.06.2018
   - erste Version

   WKLA 07.06.2023
   - neue Version
*/
#include "main.h"

#include <avr/wdt.h>

#include "Arduino.h"
#define debug

// Hardware Arduino Uno -> Zielplatform TinyTPS mit D1 Relais
// Din  0 1 2 3
// Dout 4 5 6 9
// PWM  7 8
// PRG 10, SEL 2
// Definition der Ein/Ausgabe Pins
// Ausgänge
const byte OUT_PUMP = 4;         // Ausgang für das Pumprelais
const byte LED_PUMP = 5;         // LED parallel zur Pumpe
const byte LED_TANK_FULL = 6;    // LED zeigt den Speicherstatus an
const byte LED_FILTER_FULL = 9;  // LED zeigt den Filterstand an
// Eingänge
const byte SEN_TANK_FULL = 0;    // Sensor Tank voll
const byte SEN_FILTER_FULL = 1;  // Sensor Vorfilter voll
const byte SWT_PUMP_MAN = 10;    // Taster manueller Pumpen Betrieb: active = low
const byte SWT_AUTO_MAN = 2;     // Schalter manueller Betrieb: low = man / high = auto

// Verzögerung einer Loop in msec
#define LOOP_TIME 250
const byte LOOP_COR_FACT = 1000 / LOOP_TIME;
// Nachlaufzeit der Pumpe in Sekunden
#ifdef debug
#define RUN_ON_TIME 3
#else
#define RUN_ON_TIME 30
#endif
const byte LAP_COUNT = RUN_ON_TIME * LOOP_COR_FACT;

// Autoreset, nach dieser Anzahl der Runden wird
// der Watchdog nicht mehr getriggert und das System rebootet
#ifdef debug
const byte MAX_AUTO_RESTART  = 60 * LOOP_COR_FACT;
#else
const byte MAX_AUTO_RESTART =  60 * 60 * LOOP_COR_FACT;
#endif

void setup(

) {
  // Ausgänge definieren
  pinMode(OUT_PUMP, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  pinMode(LED_TANK_FULL, OUTPUT);
  pinMode(LED_FILTER_FULL, OUTPUT);
  // Eingänge definieren
  pinMode(SEN_TANK_FULL, INPUT_PULLUP);
  pinMode(SEN_FILTER_FULL, INPUT_PULLUP);
  pinMode(SWT_PUMP_MAN, INPUT_PULLUP);
  pinMode(SWT_AUTO_MAN, INPUT_PULLUP);

  pumpOff();
  ledOff();

  wdt_enable(WDTO_4S);
}

// automatische Resetzeit
long autoRestart = MAX_AUTO_RESTART;  // einmal die Stunde, Rundenzeit ist etwas 250ms
byte c = 0;

bool tkFull, flFull, atMode, mnPump;
bool svPump;
byte ppCounter;

void loop() {
  // WatchDog verarbeiten
  doAutoRestart();
  // alle Sensoren und Taster/Schalter lesen
  readAllInputs();
  // Anzeigen verarbeiten
  doTankFull(tkFull);
  doFilterFull(flFull);

  // manueller Override der Pumpe
  doManualPump();
  // do the automatic pump operation
  doAutoPump();

  delay(LOOP_TIME);
}

// do the automatic pump operation
void doAutoPump() {
  if(atMode && !mnPump) {
    if(flFull && !tkFull) {
      ppCounter = LAP_COUNT;
    }
    if(ppCounter > 0) {
      pumpOn();
      ppCounter--;
    } else {
      pumpOff();
    }
  }
}

// manueller Override der Pumpe
void doManualPump() {
  if(mnPump) {
    if(!svPump) {
      pumpOn();
      svPump = true;
    }
  } else {
    if(svPump) {
      pumpOff();
      svPump = false;
    }
  }
}

void readAllInputs() {
  tkFull = isTankFull();
  flFull = isFilterFull();
  atMode = isAutoMode();
  mnPump = isManualPump();
}

// WatchDog triggern und nach definierter Zeit einen Reset provozieren
void doAutoRestart() {
  // Counter bis zu Reset erniedrigen
  autoRestart--;
  if(autoRestart > 0) {
    // wenn noch wartezeit übrig ist, dann den Watchdog triggern
    wdt_reset();
  } else {
    // Wartezeit verstrichen, Watchdog wird resetten
    pumpOff();
    ledOff();
    while(true) {
      // solange hektisch blinken bitte...
      digitalWrite(LED_PUMP, !digitalRead(LED_PUMP));
      delay(100);
    }
  }
}

// schalte Pumpe aus
void pumpOff() { doPump(false); }

// schalte Pumpe aus
void pumpOn() { doPump(true); }

// Alle LEDs aus
void ledOff() {
  digitalWrite(LED_PUMP, 0);
  digitalWrite(LED_TANK_FULL, 0);
  digitalWrite(LED_FILTER_FULL, 0);
}

// Ist die Hauptwassertonne schon voll?
bool isTankFull() { return !digitalRead(SEN_TANK_FULL); }

// Ist der Vorfilter schon voll?
bool isFilterFull() { return !digitalRead(SEN_FILTER_FULL); }

// Ist automatic Modus gewählt?
bool isAutoMode() { return !digitalRead(SWT_AUTO_MAN); }

// manuelle Pumpe
bool isManualPump() { return !digitalRead(SWT_PUMP_MAN); }

// Pumpe ein/ausschalten
void doPump(bool start) {
  digitalWrite(LED_PUMP, start);
  digitalWrite(OUT_PUMP, start);
}

// Signal LED "Tonne voll" de/aktivieren
void doTankFull(bool full) { digitalWrite(LED_TANK_FULL, full); }

// Signal LED "Vorfiler voll" de/aktivieren
void doFilterFull(bool full) { digitalWrite(LED_FILTER_FULL, full); }