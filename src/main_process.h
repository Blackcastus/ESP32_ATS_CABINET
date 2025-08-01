#ifndef __MAIN_PROCESS_H
#define __MAIN_PROCESS_H

#include "PZEM004Tv30.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SimpleKalmanFilter.h>



#define PZEM_DEVICE_ID_ADDR_SLAVER 0x01
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#define LCD_ID_ADDR 0x27

#define LED_DEBUD 2
#define OUTPUT_1 26
#define OUTPUT_2 27
#define ZMPT_PINOUT 34
#define ZMPT2_PINOUT 35
#define RELAY1(x) digitalWrite(OUTPUT_1, x)
#define RELAY2(x) digitalWrite(OUTPUT_2, x)



struct AC_voltage
{
  float Voltage_AC;
  float Current_AC;
  float Power_AC;
  float Frequency_AC;
};

struct Read_Analog
{
  uint16_t ADC_Value;
  float Volt_Value_Raw;
  float Volt_Value_Filter;
  float Volt_AC;
};

struct Wifi_Config
{
  String Wifi_Ssid;
  String Wifi_Pass;
  uint8_t Wifi_Status;
  
};

enum AC_STATUS
{
  OFF_ALL = 0,
  ON_AP,
  THAP_AP,
  CAO_AP,
  DU_PHONG,
};

void Peripheral_Init();
void Display_Process();
void PZEM_004T_Process();
void ZMPT_Process();
void Blynk_Process();

#endif