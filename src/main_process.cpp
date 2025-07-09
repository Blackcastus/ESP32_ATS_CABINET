#include "main_process.h"

PZEM004Tv30 PZEM_004T(Serial2, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_DEVICE_ID_ADDR_SLAVER);
LiquidCrystal_I2C lcd(LCD_ID_ADDR,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Read_Analog ZMPT_AC;
AC_voltage PZEM_AC;

SimpleKalmanFilter kf( /* mea_e=R */ 0.1,   // Measurement noise
                       /* est_e=P */ 1,     // Estimation error
                       /* q=Q      */ 0.01);// Process noise

const int numSamples = 1000;
float offset = 2048; // Trung tâm tín hiệu ADC nếu dùng độ phân giải 12 bit
float voltageRef = 3.3;
int adcResolution = 4095;

void Peripheral_Init()
{
    Serial.begin(9600);
    while(!Serial){                                                     //Waiting for USB Serial COM port to open.
    }
    pinMode(LED_DEBUD, OUTPUT);
    pinMode(OUTPUT_1, OUTPUT);
    pinMode(OUTPUT_2, OUTPUT);

    lcd.init();
    lcd.backlight();
    lcd.clear();
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
        float calibration = 666.0;  // ví dụ: 250 ứng với ~0.65V RMS đầu ra
        ZMPT_AC.Volt_AC = ZMPT_AC.Volt_Value_Filter * calibration;

        Serial.printf("Raw=%.3f  Kalman=%.3f  AC=%.1f V\n",
                vRms, ZMPT_AC.Volt_Value_Filter, ZMPT_AC.Volt_AC);

        if(ZMPT_AC.Volt_AC > 100.0)
        {
            RELAY1(1);
            RELAY2(0);
        }
        else
        {
            RELAY1(0);
            RELAY2(1);
        }
    }
}