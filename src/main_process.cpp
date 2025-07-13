#include "main_process.h"

#define BLYNK_TEMPLATE_ID           "TMPL69dErE9xA"
#define BLYNK_TEMPLATE_NAME         "ATS_CABINET"
#define BLYNK_AUTH_TOKEN            "WaOFYCZFpQdS-549wRJeeiV70eRbVPKU"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

String mWifi_Ssid = "cotsongkhongon";
String mWifi_Pass = "dunglatrung";


PZEM004Tv30 PZEM_004T(Serial2, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_DEVICE_ID_ADDR_SLAVER);
LiquidCrystal_I2C lcd(LCD_ID_ADDR,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Read_Analog ZMPT_AC;
AC_voltage PZEM_AC;

SimpleKalmanFilter kf( /* mea_e=R */ 0.1,   // Measurement noise
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
    while(!Serial){                                                     //Waiting for USB Serial COM port to open.
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
    if(millis() - Display_Interval >= 999)
    {
        Display_Interval = millis();

        // lcd.clear();
        lcd.setCursor(0, 0);
        lcd.printf("V: %0.0f V        ", (PZEM_AC.Voltage_AC + 1));
        lcd.setCursor(0, 1);
        lcd.printf("I: %0.0f mA        ", PZEM_AC.Current_AC);

        
    }
}
void PZEM_004T_Process()
{
    static uint32_t Pzem_Interval = 0;
    if(millis() - Pzem_Interval >= 999)
    {
        Pzem_Interval = millis();
        
        
        PZEM_AC.Voltage_AC = PZEM_004T.voltage();
        PZEM_AC.Current_AC = PZEM_004T.current() * 1000.0f;
        PZEM_AC.Power_AC = PZEM_004T.power();
        // Serial.printf("Volt: %0.0f V\n", PZEM_AC.Voltage_AC);
        // Serial.printf("Current: %0.0f mA\n", PZEM_AC.Current_AC);
        // Serial.printf("Powwer: %0.4f\n", PZEM_AC.Power_AC);
        // Serial.println();
    }
}
void ZMPT_Process()
{
    
    static uint32_t Zmpt_Interval = 0;
    if(millis() - Zmpt_Interval >= 5000)
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
        float calibration = 666.0;  // ví dụ: 250 ứng với ~0.65V RMS đầu ra
        ZMPT_AC.Volt_AC = ZMPT_AC.Volt_Value_Filter * calibration;

        // Serial.printf("Raw=%.3f  Kalman=%.3f  AC=%.1f V\n",
        //         vRms, ZMPT_AC.Volt_Value_Filter, ZMPT_AC.Volt_AC);
        
        uint8_t relay1_status = 0;
        uint8_t relay2_status = 0;
        if(ZMPT_AC.Volt_AC > 100.0)
        {
            relay1_status = 1;
            relay2_status = 0;
            
        }
        else
        {
            relay1_status = 0;
            relay2_status = 1;
        }
        RELAY1(relay1_status);
        RELAY2(relay2_status);

        Blynk.virtualWrite(V4, relay1_status);
        Blynk.virtualWrite(V7, relay2_status);

        uint8_t tmp_v = random(150, 250);
        uint8_t tmp_i = random(0, 10);
        if (WiFi.status() == WL_CONNECTED && Blynk.connected())
        {
            Blynk.virtualWrite(V5, tmp_v);
            Blynk.virtualWrite(V6, tmp_i);
        }
        Serial.println("Check");
    }
}

void Blynk_Process()
{
    Blynk.run();
}

// // Nhận trạng thái nút nhấn trên blynk app và điều khiển led theo trạng thái nút nhấn
BLYNK_WRITE(V0) {
    int value = param.asInt();
    digitalWrite(LED_DEBUD, value);
    Serial.printf("STATUS: %d\r\n", value);
    if (WiFi.status() == WL_CONNECTED && Blynk.connected())
    {
        Blynk.virtualWrite(V4, value);        // Cập nhật LED ảo trên app
    }
}

void SERIAL_PROCESS()
{
    if (Serial.available())
	{
		String serial_data = Serial.readString();
        serial_data.trim();
        if (WiFi.status() != WL_CONNECTED)
        {
            switch(WIFI_CONNECTED.Wifi_Status)
            {
                case 0:
                    if (strcmp(serial_data.c_str(), (char*)"wifi") == 0)
                    {
                        WIFI_CONNECTED.Wifi_Status = 1; // 
                        Serial.println("ENTER YOUR WIFI NAME:");
                    }
                    break;
                case 1:
                    WIFI_CONNECTED.Wifi_Status = 2; // 
                    WIFI_CONNECTED.Wifi_Ssid = serial_data;
                    Serial.println(WIFI_CONNECTED.Wifi_Pass);
                    Serial.println("ENTER YOUR PASSWORD:");
                    break;
                case 2:
                    WIFI_CONNECTED.Wifi_Status = 3;
                    WIFI_CONNECTED.Wifi_Pass = serial_data;
                    Serial.println(WIFI_CONNECTED.Wifi_Pass);
                    break;

                default:
                break;
            }
            
        }
    }
}

void WIFI_PROCESS()
{
    
    if (WiFi.status() != WL_CONNECTED)
    {
        if(WIFI_CONNECTED.Wifi_Status == 3)
        {
            Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_CONNECTED.Wifi_Ssid.c_str(), WIFI_CONNECTED.Wifi_Pass.c_str());
            delay(2000);
            if (WiFi.status() == WL_CONNECTED && Blynk.connected())
            {
                // WIFI_CONNECTED.Wifi_Status = 1;
                Serial.printf("WIFI: %s\r\n", mWifi_Ssid.c_str());
                Serial.printf("PASS: %s\r\n", mWifi_Pass.c_str());
            }
        }

        WIFI_CONNECTED.Wifi_Status = 0;
        
    }
}