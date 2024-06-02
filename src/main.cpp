/*
   Diese kleine Programm dient dazu eine Wassertonne mit Vorfilteranlage zu
   steuern. Folgende Funktionen übernimmt das Programm.

  Version 2
   - Wenn Vorfilter voll und Hauptspeicher nicht voll, starten einer Wasserpumpe
   mit Nachlaufzeit.
   - Pumpe wird direkt ausgeschaltet, wenn Hauptspeicher voll.
   - Watchdog falls System in einem undefinierten Zustand gerät.
   - Anzeige der Füllung auf eine Balkenanzeige (8x RGB LEDs)
   - neuer Wasserstandstsensor mit Piezo (4ma-20mA)

   Historie
   WKLA 13.07.2018
   - Watchdog implementiert
   - verschiedene Zeitkonstanten für Debug und nicht Debug version
   - Board LED als Status LED

   WKLA 16.06.2018
   - erste Version

   WKLA 07.06.2023
   - Version 2 für ATTiny84
   - Piezo-Wasserstandstsensor
   - eigene Platine

   WKLA 01.06.2024
   - Anpassungen
   - neues Layout für das LED Band: 
     LED 1-3: Poweranzeige
     LED 1:   Tank voll -> ROT
     LED 2:   Filter voll -> ROT
     LED 3:   Pumpen -> Grün
     LED4-8:  5 stufige Anzeige des Füllgrades grün, LED 8: Sensorfehler -> Rot
   - Bug in Mittelwertbildung behoben.
   - Tank voll, sofort Pumpende 
*/
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

#include "Arduino.h"
#define ledstripe
// #define debug

// Hardware Arduino Uno -> Zielplattform TinyTPS mit D1 Relais
// Din  0 1 2 3
// Dout 4 5 6 9
// PWM  7 8
// PRG 10, SEL 2
// Definition der Ein/Ausgabe Pins
// Ausgänge
const byte OUT_PUMP = 4;         // Ausgang für das Pumprelais
const byte LED_PUMP = 5;         // LED parallel zur Pumpe
const byte LED_TANK_FULL = 6;    // LED zeigt den Speicherstatus an
const byte LED_AUTO = 7;         // LED für Automatikmodus
const byte LED_STRIP_PIN = 8;    // LED Zeile für die analoge Level Ausgabe
const byte LED_FILTER_FULL = 9;  // LED zeigt den Filterstand an
// Eingänge
const byte SEN_TANK_FULL = 0;    // Sensor Tank voll
const byte SEN_FILTER_FULL = 1;  // Sensor Vorfilter voll
const byte SWT_AUTO_MAN = 2;     // Schalter manueller Betrieb: low = man / high = auto
const byte SEN_TANK_FLOAT = A3;  // Sensor Tank analoges Signal zur Tankfüllung
const byte SWT_PUMP_MAN = 10;    // Taster manueller Pumpen Betrieb: active = low

// Anzahl der LEDs im Balken
const byte LED_STRIP_COUNT = 8;

// Mindestverzögerung einer Loop in msec
// Die eigentliche Verarbeitung im Programm wird bei dieser Zeit nicht berücksichtigt
#define LOOP_TIME 100
const word ERR_LVL = 100;
const word MIN_LVL = 220;  // Wert von 4mA für den 0-Punkt
const word MAX_LVL = 942;  // 1024 / 5 * 4,6 = 942   1024 = 10 Bit A/D Auflösung = 5V (Referenzspannung) 4.6V gemessen bei max. Pegel

// Korrekturfaktor Anzahl der Runden pro Sekunde
const long LOOP_COR_FACT = 1000 / LOOP_TIME;
// Nachlaufzeit der Pumpe in Sekunden
#ifdef debug
#define RUN_ON_TIME 3
#else
#define RUN_ON_TIME 15
#endif

// Helligkeit der Balkenanzeige
#define BRIGHTNESS 10

// calculating constants
// Nachlaufzeit der Pumpe in loop Zyklen
const byte PUMP_LAP_COUNT = RUN_ON_TIME * LOOP_COR_FACT;

// Autoreset, nach dieser Anzahl der Runden wird
// der Watchdog nicht mehr getriggert und das System rebooted automatisch
#ifdef debug
const long MAX_AUTO_RESTART = 60 * LOOP_COR_FACT;
#else
const long MAX_AUTO_RESTART = 60L * 60L * LOOP_COR_FACT;
#endif

// Anzahl der gespeicherten Levelwerte
const byte MAX_LVLS = 7;
byte lvls[MAX_LVLS];
byte pos;

#ifdef ledstripe
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
uint32_t LED_BLACK = strip.Color(0, 0, 0);
uint32_t LED_GREEN = strip.Color(0, 255, 0);
uint32_t LED_RED = strip.Color(255, 0, 0);
uint32_t LED_BLUE = strip.Color(0, 0, 255);
#endif

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
void initAvr();

void setup() {
  // Ausgänge definieren
  pinMode(OUT_PUMP, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  pinMode(LED_TANK_FULL, OUTPUT);
  pinMode(LED_FILTER_FULL, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);
  pinMode(LED_STRIP_PIN, OUTPUT);
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

  initAvr();

// Anzeige initialisieren
#ifdef ledstripe
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
#endif
}

// automatische Resetzeit
long autoRestart = MAX_AUTO_RESTART;  // einmal die Stunde, Rundenzeit ist etwas 250ms
byte c = 0;

bool tkFull, flFull, atMode, mnPump, pump;
bool svPump;
bool lvlerr;
byte ppCounter;
byte tkLvl;

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
  // Mindestwartezeit eines Durchlauf
  delay(LOOP_TIME);

#ifndef ledstripe
  digitalWrite(LED_STRIP_PIN, !digitalRead(LED_STRIP_PIN));
#endif
}

// do the automatic pump operation
void doAutoPump() {
  digitalWrite(LED_AUTO, !atMode);
  if(atMode) {
    if((flFull || mnPump) && !tkFull) {
      ppCounter = PUMP_LAP_COUNT;
    }
    if (tkFull) {
      ppCounter = 0;
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
    // Wartezeit verstrichen, Watchdog löst nun den Reset aus
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
  pos = byte(((pos + 1) % MAX_LVLS));
  // building the average
  word sum = 0;
  byte min, max;
  min = 100;
  max = 0;
  // sum all up, determine min and max
  for(byte i = 0; i < MAX_LVLS; i++) {
    sum += lvls[i];
    if(lvls[i] < min) {
      min = lvls[i];
    }
    if(lvls[i] > max) {
      max = lvls[i];
    }
  }
  // remove the min and the max from the sum
  sum -= (min + max);
  // build average, divide the sum with the count of measure points minus 2 (min and max)
  return byte(sum / (MAX_LVLS - 2));
}

// initialise the average building array
void initAvr() {
    for(byte i = 0; i < MAX_LVLS; i++) {
    lvls[i] = 0;
  }
  pos = 0;
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
#ifdef ledstripe
  strip.clear();
  strip.show();
#endif
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

// Signal LED "Vorfilter voll" de/aktivieren
void doFilterFull(bool full) { digitalWrite(LED_FILTER_FULL, full); }

void doStrip() {
#ifdef ledstripe
  for(byte i = 0; i < 3; i++) {
    strip.setPixelColor(i, strip.Color(32, 32, 32));
  }
  if(lvlerr) {
    for(int8_t i = 0; i < 5; i++) {
      strip.setPixelColor(LED_STRIP_COUNT - i - 1, LED_BLACK);
    }
      strip.setPixelColor(7, LED_RED);
  } else {
    int8_t lvl = map(tkLvl, 0, 100, -1, 5);
    for(int8_t i = 0; i < 5; i++) {
      if(i <= lvl) {
        strip.setPixelColor(LED_STRIP_COUNT - i - 1, LED_GREEN);
      } else {
        strip.setPixelColor(LED_STRIP_COUNT - i - 1, LED_BLACK);
      }
    }
  }
  if(tkFull) {
    strip.setPixelColor(2, LED_RED);
  }
  if(flFull) {
    strip.setPixelColor(1, LED_RED);
  }
  if(pump || mnPump) {
    strip.setPixelColor(0, LED_GREEN);
  }
  strip.show();
#endif
}