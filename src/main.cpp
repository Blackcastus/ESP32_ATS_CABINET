#include <Arduino.h>
#include "main_process.h"

void setup() {
  Peripheral_Init();
}

void loop() {
  PZEM_004T_Process();
  ZMPT_Process();
  Display_Process();
  Blynk_Process();
  
}

