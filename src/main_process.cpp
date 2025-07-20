#include "main_process.h"

// #define BLYNK_TEMPLATE_ID           "TMPL69dErE9xA"
// #define BLYNK_TEMPLATE_NAME         "ATS_CABINET"
#define BLYNK_TEMPLATE_ID "TMPL6Y8O6dnEQ"
#define BLYNK_TEMPLATE_NAME "LED ESP32"
#define BLYNK_AUTH_TOKEN "iRqj1v8a_qW8iKUpgIAUjALpCXAzjoCZ"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

String mWifi_Ssid = "HPhuc";
String mWifi_Pass = "03032003";


PZEM004Tv30 PZEM_004T(Serial2, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_DEVICE_ID_ADDR_SLAVER);
LiquidCrystal_I2C lcd(LCD_ID_ADDR,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Read_Analog ZMPT_AC;
Read_Analog ZMPT_AC2;
AC_voltage PZEM_AC;
uint8_t relay1_status = 0;
uint8_t relay2_status = 0;
uint8_t Status_AC     = OFF_ALL;
uint8_t Status_AC_LAST     = OFF_ALL;

SimpleKalmanFilter kf( /* mea_e=R */ 0.1,   // Measurement noise
                       /* est_e=P */ 1,     // Estimation error
                       /* q=Q      */ 0.01);// Process noise

SimpleKalmanFilter kf1( /* mea_e=R */ 0.1,   // Measurement noise
                       /* est_e=P */ 1,     // Estimation error
                       /* q=Q      */ 0.01);// Process noise

Wifi_Config WIFI_CONNECTED;

const int numSamples = 1000;
float offset = 2048; // Trung tâm tín hiệu ADC nếu dùng độ phân giải 12 bit
float voltageRef = 3.3;
int adcResolution = 4095;


void Peripheral_Init()
{
    Serial.begin(115200);
    while(!Serial){
    }

    Blynk.begin(BLYNK_AUTH_TOKEN, mWifi_Ssid.c_str(), mWifi_Pass.c_str());
    // delay(2000);
    if (WiFi.status() == WL_CONNECTED)
    {
        // WIFI_CONNECTED.Wifi_Status = 1;
        Serial.printf("WIFI: %s\r\n", mWifi_Ssid.c_str());
        Serial.printf("PASS: %s\r\n", mWifi_Pass.c_str());
    }
    else
    {
        // WIFI_CONNECTED.Wifi_Status = 0;
        Serial.println("WIFI NOT AVAILABLE!!!");
    }
    pinMode(LED_DEBUD, OUTPUT);
    pinMode(OUTPUT_1, OUTPUT);
    pinMode(OUTPUT_2, OUTPUT);

    lcd.init();
    lcd.backlight();
    lcd.clear();
    Serial.println("ATS CABINET STATED");
}

void Display_Process()
{
    static uint32_t Display_Interval = 0;
    if(millis() - Display_Interval >= 100)
    {
        Display_Interval = millis();

        // lcd.clear();
        // lcd.clear();
        lcd.setCursor(0, 0);
        lcd.printf("V: %0.0f V        ", (PZEM_AC.Voltage_AC + 1));
        lcd.setCursor(0, 1);
        lcd.printf("I: %0.0f mA        ", PZEM_AC.Current_AC);
        if(Status_AC_LAST != Status_AC)
        {
            Status_AC_LAST = Status_AC;
            switch(Status_AC)
            {
                case ON_AP:
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.printf("V: %0.0f V        ", (ZMPT_AC.Volt_AC));
                    lcd.setCursor(0, 1);
                    lcd.printf("ON AP       ");
                break;
                case THAP_AP:
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.printf("V: %0.0f V        ", (ZMPT_AC.Volt_AC));
                    lcd.setCursor(0, 1);
                    lcd.printf("THAP AP       ");
                break;
                case CAO_AP:
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.printf("V: %0.0f V        ", (ZMPT_AC.Volt_AC));
                    lcd.setCursor(0, 1);
                    lcd.printf("CAO AP       ");
                break;
                case DU_PHONG:
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.printf("V: %0.0f V        ", (ZMPT_AC2.Volt_AC));
                    lcd.setCursor(0, 1);
                    lcd.printf("NGUON DU PHONG");
                break;
                default:
                    lcd.clear();
                    lcd.printf("MAT AP       ");
                break;
            }
            delay(2000);
            lcd.clear();
        }
    }
}
void PZEM_004T_Process()
{
    static uint32_t Pzem_Interval = 0;
    if(millis() - Pzem_Interval >= 999)
    {
        Pzem_Interval = millis();
        
        float volt = PZEM_004T.voltage();
        float amp = PZEM_004T.current();
        PZEM_AC.Voltage_AC = volt == -1? 0: volt;
        PZEM_AC.Current_AC = amp == -1? 0: amp * 1000.0f;
        // PZEM_AC.Power_AC = PZEM_004T.power();
        // Serial.printf("Volt: %0.0f V\n", PZEM_AC.Voltage_AC);
        // Serial.printf("Current: %0.0f mA\n", PZEM_AC.Current_AC);
        // Serial.printf("Powwer: %0.4f\n", PZEM_AC.Power_AC);
        // Serial.println();

    }
}
void ZMPT_Process()
{
    
    static uint32_t Zmpt_Interval = 0;
    if(millis() - Zmpt_Interval >= 10)
    {
        Zmpt_Interval = millis();

        long sum = 0;
        for (int i = 0; i < numSamples; i++) {
            int val = analogRead(ZMPT_PINOUT);
            long diff = val - offset;
            sum += diff * diff;
        }

        float mean = sum / (float)numSamples;
        float rms = sqrt(mean);

        // Chuyển ADC RMS sang điện áp RMS
        float vRms = (rms / adcResolution) * voltageRef;
        // float vRmsFiltered = 0;
        if(vRms > 0.1)
        {
            ZMPT_AC.Volt_Value_Filter = kf.updateEstimate(vRms);
        }
        else
        {
            ZMPT_AC.Volt_Value_Filter = 0;
        }
        // Hệ số hiệu chỉnh tuỳ vào mạch ZMPT và trở tinh chỉnh
        float calibration = 680.0;  // ví dụ: 250 ứng với ~0.65V RMS đầu ra
        ZMPT_AC.Volt_AC = ZMPT_AC.Volt_Value_Filter * calibration;
        
        long sum2 = 0;
        for (int i = 0; i < numSamples; i++) {
            int val = analogRead(ZMPT2_PINOUT);
            long diff = val - offset;
            sum2 += diff * diff;
        }

        float mean2 = sum2 / (float)numSamples;
        float rms2 = sqrt(mean2);

        // Chuyển ADC RMS sang điện áp RMS
        float vRms2 = (rms2 / adcResolution) * voltageRef;
        // float vRmsFiltered = 0;
        if(vRms2 > 0.1)
        {
            ZMPT_AC2.Volt_Value_Filter = kf1.updateEstimate(vRms2);
        }
        else
        {
            ZMPT_AC2.Volt_Value_Filter = 0;
        }
        // Hệ số hiệu chỉnh tuỳ vào mạch ZMPT và trở tinh chỉnh
        // float calibration = 666.0;  // ví dụ: 250 ứng với ~0.65V RMS đầu ra
        calibration = 610;
        ZMPT_AC2.Volt_AC = ZMPT_AC2.Volt_Value_Filter * calibration;
    }

    
    if(ZMPT_AC.Volt_AC > 215.0) // ổn áp
    {
        relay1_status = 1;
        relay2_status = 0;
        Status_AC = ON_AP;
    }
    else if((ZMPT_AC.Volt_AC > 50.0 && ZMPT_AC.Volt_AC <= 215.0) || ZMPT_AC.Volt_AC >= 240.0) // Thấp áp và cao áp
    {
        relay1_status = 0;
        relay2_status = 1;
        if(ZMPT_AC.Volt_AC <= 215.0)
        {
            Status_AC = THAP_AP;
        }
        else
        {
            Status_AC = CAO_AP;
        }
    }
    else
    {
        if(ZMPT_AC2.Volt_AC > 215)
        {
            relay1_status = 0;
            relay2_status = 1;
            Status_AC = DU_PHONG;

        }
        else
        {
            relay1_status = 0;
            relay2_status = 0;
            Status_AC = OFF_ALL;
        }
    }
    RELAY1(relay1_status);
    RELAY2(relay2_status);
}

void Blynk_Process()
{
    Blynk.run();
    static uint32_t Blink_Interval = 0;
    if(millis() - Blink_Interval >= 2999)
    {
        Blink_Interval = millis();
        if (WiFi.status() == WL_CONNECTED && Blynk.connected())
        {
        Blynk.virtualWrite(V5, PZEM_AC.Voltage_AC);
        Blynk.virtualWrite(V6, PZEM_AC.Current_AC);
        Blynk.virtualWrite(V4, relay1_status);
        Blynk.virtualWrite(V7, relay2_status);
        Blynk.virtualWrite(V8, ZMPT_AC.Volt_AC);
        Blynk.virtualWrite(V9, ZMPT_AC2.Volt_AC);
        // Serial.printf(": %0.2f\r\n", PZEM_AC.Voltage_AC);
        // Serial.printf(": %0.2f\r\n", PZEM_AC.Current_AC);
        // Serial.printf("nguon chinh: %0.2f\r\n", ZMPT_AC.Volt_AC);
        // Serial.printf("Nguon du phong: %0.2f\r\n", ZMPT_AC2.Volt_AC);
        switch(Status_AC)
        {
            case 1:
                Blynk.virtualWrite(V0, "NGUỒN LƯỚI ỔN ÁP\r\n");
            break;
            case 2:
                Blynk.virtualWrite(V0, "NGUỒN LƯỚI THẤP ÁP\r\n");
            break;
            case 3:
                Blynk.virtualWrite(V0, "NGUỒN LƯỚI CAO ÁP\r\n");
            break;
            default:
                Blynk.virtualWrite(V0, "MẤT ĐIỆN ÁP\r\n");
            break;
        }
        }
    }
}