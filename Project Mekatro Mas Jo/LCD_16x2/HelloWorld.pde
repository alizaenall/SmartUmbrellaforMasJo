//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int conditionStateP = 0;
int conditionStateL = 0;
void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");
  lcd.setCursor(2,1);
  lcd.print("ALI");
  delay(1000);
  lcd.print("Zaenal");
}

void lcdDisplay(){
  Serial.println("LCD DISPLAY");
  switch(conditionStateP){
    case 1:
      lcd.setCursor(6,1);
      lcd.print("PANAS");
      break;
    case 2:
      lcd.setCursor(6,1);
      lcd.print("LEBAT");
      break;
    case 3:
      lcd.setCursor(6,1);
      lcd.print("SEDANG");
      break;
    case 4:
      lcd.setCursor(6,1);
      lcd.print("RINGAN");
      break;
    case 5:
    Serial.println("SWITHC CASE 5");
      lcd.setCursor(6,1);
      lcd.print("NoRain");
      break;
  }
  switch(conditionStateL){
    case 1:
      lcd.setCursor(14,1);
      lcd.print("GLP");
      break;
    case 2:
      Serial.println("SWITHC CASE 2");
      lcd.setCursor(14,1);
      lcd.print("TRG");
    break;
  }
}
void loop(){
  conditionStateP++;
  conditionStateL++;
  lcdDisplay();
  delay(1000);
  
}
