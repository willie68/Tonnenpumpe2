/*
   Diese kleine Programm dient dazu eine Wassertonne mit Vorfilteranlage zu
   steuern. Folgende Funktionen übernimmt das Programm.

  Version 2
   - Wenn Vorfilter voll und Hauptspeicher nicht voll, starten einer Wasserpumpe
   mit Nachlaufzeit.
   - Pumpe wird direkt ausgeschaltet, wenn Hauptspeicher voll.
   - Watchdog falls System in einem undefinerten Zustand gerät.
   - Anzeige der Füllung auf einee Balkenanzeige (8x RGB LEDs)
   - neuer Wasserstandssensor mit Piezo

   Historie
   WKLA 13.07.2018
   - Watchdog implementiert
   - verschiedene Zeitkonstanten für Debug und nicht Debug version
   - Board LED als Status LED

   WKLA 16.06.2018
   - erste Version

   WKLA 07.06.2023
   - Version 2 für ATTiny84
   - Piezo-Wasserstandssensor
   - eigene Platine
*/
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

#include "Arduino.h"
// #define debug

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
const byte LED_AUTO = 7;         // LED für utomatikmodus
const byte LED_STRIP_PIN = 8;    // LED Zeile für die analoge Level Ausgabe
// Eingänge
const byte SEN_TANK_FULL = 0;    // Sensor Tank voll
const byte SEN_FILTER_FULL = 1;  // Sensor Vorfilter voll
const byte SWT_PUMP_MAN = 10;    // Taster manueller Pumpen Betrieb: active = low
const byte SWT_AUTO_MAN = 2;     // Schalter manueller Betrieb: low = man / high = auto
const byte SEN_TANK_FLOAT = A3;  // Sensor Tank analoges Signal zur Tankfüllung

// Anzahl der LEDs im Balken
const byte LED_STRIP_COUNT = 8;

// Mindestverzögerung einer Loop in msec
// Die eigentliche Verarbeitung im Programm wird bei dieser Zeit nicht berücksichtigt
#define LOOP_TIME 100
const word ERR_LVL = 100;
const word MIN_LVL = 220;
const word MAX_LVL = 660;

// Korrekturfaktor Anzahl der Runden pro Sekunde
const byte LOOP_COR_FACT = 1000 / LOOP_TIME;
// Nachlaufzeit der Pumpe in Sekunden
#ifdef debug
#define RUN_ON_TIME 3
#else
#define RUN_ON_TIME 15
#endif

// Helligkeit der Balkenanzeige
#define BRIGHTNESS 10

// calculating constants
// Nachlaufzeit der Pumpe in loop zyklen
const byte PUMP_LAP_COUNT = RUN_ON_TIME * LOOP_COR_FACT;

// Autoreset, nach dieser Anzahl der Runden wird
// der Watchdog nicht mehr getriggert und das System rebootet automatisch
#ifdef debug
const long MAX_AUTO_RESTART = 60 * LOOP_COR_FACT;
#else
const long MAX_AUTO_RESTART = 60L * 60L * LOOP_COR_FACT;
#endif

// anzahl der gespeicherten Levelwerte
const byte MAX_LVLS = 7;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

void doAutoPump();
void doManualPump();
void readAllInputs();
void doAutoRestart();
byte getTankLevel();
void pumpOff();
void pumpOn();
void ledOff();
bool isTankFull();
bool isFilterFull();
bool isAutoMode();
bool isManualPump();
void doPump(bool);
void doTankFull(bool);
void doFilterFull(bool);
void doStrip();
byte getAverage(byte);

void setup() {
  // Ausgänge definieren
  pinMode(OUT_PUMP, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  pinMode(LED_TANK_FULL, OUTPUT);
  pinMode(LED_FILTER_FULL, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);
  // Eingänge definieren
  pinMode(SEN_TANK_FULL, INPUT_PULLUP);
  pinMode(SEN_FILTER_FULL, INPUT_PULLUP);
  pinMode(SWT_PUMP_MAN, INPUT_PULLUP);
  pinMode(SWT_AUTO_MAN, INPUT_PULLUP);
  pinMode(SEN_TANK_FLOAT, INPUT);

  pumpOff();
  ledOff();

  // Watchdog einschalten
  wdt_enable(WDTO_4S);

  // Anzeige initialisieren
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
}

// automatische Resetzeit
long autoRestart = MAX_AUTO_RESTART;  // einmal die Stunde, Rundenzeit ist etwas 250ms
byte c = 0;

bool tkFull, flFull, atMode, mnPump, pump;
bool svPump;
bool lvlerr;
byte ppCounter;
byte tkLvl;
byte lvls[MAX_LVLS];
byte pos;

void loop() {
  // WatchDog verarbeiten
  doAutoRestart();
  // alle Sensoren und Taster/Schalter lesen
  readAllInputs();
  // Sensoren verarbeiten
  doTankFull(tkFull);
  doFilterFull(flFull);

  // manueller Override der Pumpe
  doManualPump();
  // automatisches Pumpen
  doAutoPump();
  // Ausgabe der aktuellen Messungen auf dem Balken
  doStrip();
  // mindest Wartezeit eines Durchlauf
  delay(LOOP_TIME);

  for(byte i = 0; i < MAX_LVLS; i++) {
    lvls[i] = 0;
  }
  pos = 0;
}

// do the automatic pump operation
void doAutoPump() {
  digitalWrite(LED_AUTO, !atMode);
  if(atMode) {
    if((flFull || mnPump) && !tkFull) {
      ppCounter = PUMP_LAP_COUNT;
    }
    if(ppCounter > 0) {
      pump = true;
      pumpOn();
      ppCounter--;
    } else {
      pump = false;
      pumpOff();
    }
  }
}

// manueller Override der Pumpe
void doManualPump() {
  if(!atMode) {
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
}

void readAllInputs() {
  tkFull = isTankFull();
  flFull = isFilterFull();
  atMode = isAutoMode();
  mnPump = isManualPump();
  tkLvl = getTankLevel();
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

// getting the average tank level
byte getTankLevel() {
  lvlerr = false;
  word lvl = analogRead(SEN_TANK_FLOAT);
  if(lvl < ERR_LVL) {
    lvlerr = true;
    return 0;
  }
  if(lvl < MIN_LVL) {
    return 0;
  }
  byte percent = byte(map(lvl, MIN_LVL, MAX_LVL, 0, 100));
  return getAverage(percent);
}

// calculate the average without the min and max value of the last n level measurements
byte getAverage(byte newValue) {
  lvls[pos] = newValue;
  // next position within the array
  pos = byte(((pos+1) % MAX_LVLS));
  // building the average
  word sum = 0;
  byte min, max;
  min = 100;  max  = 0;
  // sum all up, determine min and max
  for (byte i = 0; i < MAX_LVLS; i++) {
    sum += lvls[i];
    if (lvls[i] < min) {
      min = lvls[i];
    }
    if (lvls[i] > max) {
      max = lvls[i];
    }
  }
  // remove the min and the max from the sum
  sum -= (min + max);
  // build average, divide the sum with the count of measurepoints minus 2 (min and max)
  return byte(sum / (MAX_LVLS-2));
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
  digitalWrite(LED_AUTO, 0);
  strip.clear();
  strip.show();
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

void doStrip() {
  if(lvlerr) {
    strip.setPixelColor(1, strip.Color(255, 0, 0));
  } else {
    byte lvl = map(tkLvl, 0, 100, 0, LED_STRIP_COUNT);
    for(byte i = 0; i < LED_STRIP_COUNT; i++) {
      if(lvlerr) {
        strip.setPixelColor(LED_STRIP_COUNT - i - 1, strip.Color(255, 0, 0));
      } else {
        if(i <= lvl) {
          strip.setPixelColor(LED_STRIP_COUNT - i - 1, strip.Color(0, 255, 0));
        } else {
          strip.setPixelColor(LED_STRIP_COUNT - i - 1, 0);
        }
      }
    }
  }
  if(tkFull) {
    strip.setPixelColor(0, strip.Color(255, 0, 0));
  }
  if(flFull) {
    strip.setPixelColor(0, strip.Color(255, 0, 0));
  }
  if(pump) {
    strip.setPixelColor(LED_STRIP_COUNT - 1, strip.Color(0, 0, 255));
  }
  strip.show();
}