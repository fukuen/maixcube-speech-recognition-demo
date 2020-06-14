#include <Arduino.h>
#include <Sipeed_ST7789.h>
#include <Wire.h>
#include "MaixCube_Speech_Recognition.h"
#include "voice_model.h"
#include "lcd.h"

//DMAC usage
//DMAC_CHANNEL0 : FFT
//DMAC_CHANNEL1 : FFT
//DMAC_CHANNEL2 : ES8374_RX
//DMAC_CHANNEL3 : LCD
//DMAC_CHANNEL4 : ES8374_TX

#define WIDTH 240
#define HEIGHT 240
#define AXP173_ADDR 0x34
#define LED_R       13
#define LED_G       12
#define LED_B       14

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(WIDTH, HEIGHT, spi_);

MaixCubeSpeechRecognizer rec;

void drawText(char* str)
{
    lcd.fillRect(0, 0, WIDTH, HEIGHT, COLOR_BLACK);
    lcd.setCursor(16, 32);
    lcd.setTextSize(2);
    lcd.printf(str);
}

void lcd_printf(char* str)
{
    drawText(str);
	return;
}

void axp173_init()
{
    Wire.begin((uint8_t) SDA, (uint8_t) SCL, 400000);
    Wire.beginTransmission(AXP173_ADDR);
    int err = Wire.endTransmission();
    if (err)
    {
        Serial.printf("Power management ic not found.\n");
        return;
    }
    Serial.printf("AXP173 found.\n");
    // Clear the interrupts
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x46);
    Wire.write(0xFF);
    Wire.endTransmission();
    // set target voltage and current of battery(axp173 datasheet PG.)
    // charge current (default)780mA -> 190mA
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x33);
    Wire.write(0xC1);
    Wire.endTransmission();
    // REG 10H: EXTEN & DC-DC2 control
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(AXP173_ADDR, 1, 1);
    int reg = Wire.read();
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.write(reg & 0xFC);
    Wire.endTransmission();
}

void setup()
{ 
    pll_init();
    plic_init();
    dmac_init();
    uarths_init();
    Serial.begin(115200);
    axp173_init();

    lcd.begin(15000000, COLOR_BLACK);
    lcd.setRotation(2); // 
    tft_write_command(INVERSION_DISPALY_ON);

// demo1
//    rec.begin();
//    Serial.begin(115200);
//    Serial.println("start rec...");
//    if( rec.record(0, 0) == 0) //keyword_num, model_num 
//    {    
//      rec.print_model(0, 0);
//    }
//    else 
//        Serial.println("rec failed");

// demo2
    pinMode(LED_R, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_G, OUTPUT);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    rec.begin();
    Serial.begin(115200);
    Serial.println("init model...");
    drawText("init model...");

//    uint8_t reg = 0;
//    int res = es8374_read_reg(0x6e, &reg); //flag
//    Serial.printf("reg %u", reg);

    rec.addVoiceModel(0, 0, red_0, fram_num_red_0); 
    rec.addVoiceModel(0, 1, red_1, fram_num_red_1); 
    rec.addVoiceModel(0, 2, red_2, fram_num_red_2); 
    rec.addVoiceModel(0, 3, red_3, fram_num_red_3); 
    rec.addVoiceModel(1, 0, green_0, fram_num_green_0);     
    rec.addVoiceModel(1, 1, green_1, fram_num_green_1);     
    rec.addVoiceModel(1, 2, green_2, fram_num_green_2);     
    rec.addVoiceModel(1, 3, green_3, fram_num_green_3);     
    rec.addVoiceModel(2, 0, blue_0, fram_num_blue_0);   
    rec.addVoiceModel(2, 1, blue_1, fram_num_blue_1);   
    rec.addVoiceModel(2, 2, blue_2, fram_num_blue_2);   
    rec.addVoiceModel(2, 3, blue_3, fram_num_blue_3);   
    rec.addVoiceModel(3, 0, turnoff_0, fram_num_turnoff_0);  
    rec.addVoiceModel(3, 1, turnoff_1, fram_num_turnoff_1);  
    rec.addVoiceModel(3, 2, turnoff_2, fram_num_turnoff_2);  
    rec.addVoiceModel(3, 3, turnoff_3, fram_num_turnoff_3);  
    Serial.println("init model ok!");
    drawText("init model ok!");
}

void loop()
{
    int res;
    res = rec.recognize();
    Serial.printf("res : %d ", res);
    drawText("");
    lcd.setCursor(16, 32);
    lcd.printf("res : %d ", res);
    if (res > 0){
        switch (res)
        {
        case 1:
            digitalWrite(LED_R, LOW); //power on red led
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, HIGH);
            Serial.println("rec : red ");
            drawText("rec : red ");
            break;
        case 2:
            digitalWrite(LED_G, LOW); //power on green led
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_B, HIGH);
            Serial.println("rec : green ");
            drawText("rec : green ");
            break;
        case 3:
            digitalWrite(LED_B, LOW); //power on blue led
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, HIGH);
            Serial.println("rec : blue ");
            drawText("rec : blue ");
            break;
        case 4:
            digitalWrite(LED_R, HIGH);
            digitalWrite(LED_G, HIGH);
            digitalWrite(LED_B, HIGH);   //power off all leds
            Serial.println("rec : turnoff ");
            drawText("rec : turnoff ");
        default:
            break;
        }
    }else
    {
        Serial.println("recognize failed.");
        drawText("recognize failed.");
    }
    delay(1000);
}
