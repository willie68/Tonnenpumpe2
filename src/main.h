#include "Arduino.h"

void doAutoPump();
void doManualPump();
void readAllInputs();
void doAutoRestart();
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
byte getTankLevel();
void doStrip();