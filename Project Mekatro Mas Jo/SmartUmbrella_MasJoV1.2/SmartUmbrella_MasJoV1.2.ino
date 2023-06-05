#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h> 
#include <Stepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//KALIBRASI DAN ADJUST SENSOR PARAMETER DI SINI
int temperatureParameter = 32;
int rainParameter1 = 700;
int rainParameter2 = 500;
int rainParameter3 = 300;
int lightParameter = 700;
int motorRotation0 = 0;   //motor diam
int motorRotation1 = 2;   //motor berputar 1 kali
int motorRotation2 = 4;   //motor berputar 2 kali
int motorRotation3 = 6;   //motor berputar 3 kali

// STEPPER MOTOR
// Define the number of steps per revolution
const int stepsPerRevolution = 2048;
// Initialize the stepper motor with the appropriate pins
Stepper stepperName = Stepper(stepsPerRevolution, 8, 10, 9, 11);

//DHT 11 SENSOR
#define DHTPIN 2        // Digital pin connected to the DHT sensor 
#define DHTTYPE DHT11   //DHT11 sensor +-5% accuracy
//Make an object for DHT library
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// SENSOR PIN
//#define pirSensor  3              //HIGH and LOW value of PIR sensor
// #define ir_right_pin  4
// #define ir_left_pin  5
#define lightSensor A0
#define rainSensor A1
// LED PIN
#define ledTemp 7
// #define ledPir 8
// #define ledIr1 9
// #define ledIr2 10
#define ledLight 3
#define ledRain 12

#define ledPayung 6
#define ledLamp 13

#define DELAYTIMEOUT 1500

// Sensor State Variabel
int tempState = 0;              //Mendeteksi suhu
int pirState = 0;               //Mendeteksi orang

int lightState = 0;             //Mendeteksi cahaya
int rainState = 0;              //Mendeteksi hujan
int conditionStateP = 0;
int conditionStateL = 0;
char conditionStringP[30] = "-";
char conditionStringL[30] = "-"; 
//Variable
int tempValue = 0;
int pirValue = 0;
int irRightValue = 0;
int irLeftValue = 0;
int lightValue = 0;
int rainValue = 0;

//TIMING Variable
unsigned long currentTempTime = 0;
unsigned long prevTempTime = 0;
unsigned long currentPrintTime = 0;
unsigned long prevPrintTime = 0;
unsigned long currentReadTime = 0;
unsigned long prevReadTime = 0;
//unsigned long tm;
// //BIDIRECTIONAL Counting Variable
// int count = 0;
// int countState = 0;
int irRightState = 0;               //Mendeteksi obstacle 1 (kiri)
int irLeftState = 0;                //Mendeteksi obstacle 2 (kanan)
// int irRightStateLast = -1;
// int irLeftStateLast = -1;
// int inCounter = 0;
// int outCounter = 0;
// bool bWalkIn = false;
// bool bWalkOut = false;


//-----------IR SENSOR-------------//
#define DELAY_TIMEOUT 1500

int ir_right_pin = 5;
int ir_left_pin = 4;
int ir_right_state = 0;
int ir_left_state  = 0;
int ir_right_state_last = -1;
int ir_left_state_last  = -1;
int in_counter = 0;
int out_counter = 0;
bool bWalkIn = false;
bool bWalkOut = false;
unsigned long tm;
int people_detected = 0;
//-------------------------------//

//MOTOR Variable
int count = 0;
int motorFlag = 0;
int cp_then;
int rev_then;
int rev;
int rev_now;


//LCD 16x2
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,110,0);  // set the LCD address to 0x27 for a 16 chars and 2 line display




void setup() {
  //Initiate the serial monitor
  Serial.begin(9600);
  // Initialize Sensor PIN
  //PIN DHT11 diinitialize dengan library   //PIN 2
//  pinMode(pirSensor, INPUT);                //PIN 3
  pinMode(ir_right_pin, INPUT);                //PIN 4
  pinMode(ir_left_pin, INPUT);                //PIN 5
  pinMode(lightSensor, INPUT);              //PIN A0
  pinMode(rainSensor, INPUT);               //PIN A1
  //LED INdicator
  pinMode(ledTemp, OUTPUT);
  // pinMode(ledPir, OUTPUT);
  // pinMode(ledIr1, OUTPUT);
  // pinMode(ledIr2, OUTPUT);
  // pinMode(ledLight, OUTPUT);
  pinMode(ledRain, OUTPUT);
  
  //STEPPER MOTOR
  // Set the speed of the stepper motor
  stepperName.setSpeed(5);

  //Inititate DHT sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  //LCD 16x2
  lcd.init();                      // initialize the lcd 
  lcd.backlight();                 //turn on the lcd backlight
  // Print a message to LCD
  lcd.setCursor(1,0);
  lcd.print("SMART UMBRELLA");
  // lcd.setCursor(2,0);
  // lcd.print("Mechatronic!");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("People= ");
  // lcd.setCursor(0,1);
  // lcd.print("Case= ");
  lcd.setCursor(11,0);
  lcd.print("|");
  lcd.setCursor(13,0);
  lcd.print("|");
  // lcd.setCursor(0,1);
  // lcd.print("CASE= ");
}

void sensorReading(){
  // DHT11
  currentTempTime = millis();                   //untuk time delay-nya
  if(currentTempTime - prevTempTime >= 1000){   //untuk time delay-nya
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    tempValue = event.temperature;     // Reading the environment temperature

    prevTempTime = currentTempTime;
  }
  currentReadTime = millis();                 //untuk time delay-nya
  if(currentReadTime - prevReadTime >= 50){   //untuk time delay-nya
  // PIR
  //pirValue = digitalRead(pirSensor); // Reading the presence of human infrared
  // // IR1
  irRightValue = digitalRead(ir_right_pin);     //Membaca output sensor
  // // IR2
  irLeftValue = digitalRead(ir_left_pin);       //Membaca output sensor
  // LDR
  lightValue = analogRead(lightSensor);         //Membaca output sensor
  // RAIN 
  rainValue = analogRead(rainSensor);           //Membaca output sensor

  //FOR BIDIRECTIONAL ALGORITHM
  ir_right_state = digitalRead( ir_right_pin );   //Membaca output sensor
  ir_left_state =  digitalRead( ir_left_pin );    //Membaca output sensor
  
  prevReadTime = currentReadTime;                 //untuk time delay-nya
  }
}

void stateAndLamp(){
  //DHT
  if(tempValue >= temperatureParameter){                            //ADJUST sesuai kebutuhann parameter suhu
    tempState = 1;                                //untuk menjadikan parameternya sebagai nilai 0 dan 1
    digitalWrite(ledTemp, HIGH);                  // led sebagai indikator
  }
  else{
    tempState = 0;                                //untuk menjadikan parameternya sebagai nilai 0 dan 1
    digitalWrite(ledTemp, LOW);                   // led sebagai indikator
  }

  // //PIR
  // if(pirValue == 1){
  //   pirState = 0;
  //   //digitalWrite(ledPir, HIGH);
  // }
  // else{
  //   pirState = 1;
  //   //digitalWrite(ledPir, LOW);
  // }

  // //IR 1
  // if(irRightValue == 0){                
  //   irRightState = 0;                 //untuk menyalakan led 
  //   //digitalWrite(ledIr1, HIGH);
  // }
  // else{ 
  //   irRightState = 1;                 //untuk menyalakan led
  //   //digitalWrite(ledIr1, LOW);
  // }

  // //IR 2
  // if(irLeftValue == 0){
  //   irLeftState = 1;
  //   //digitalWrite(ledIr2, HIGH);
  // }
  // else{
  //   irRightState = 0;
  //   //digitalWrite(ledIr2, LOW);
  // }

  // LDR
  if(lightValue >= lightParameter){                    //untuk menjadikan parameter sebagai nilai 0 dan 1              
    lightState = 1;               // GELAP
    digitalWrite(ledLight, HIGH);           //untuk menyalakan LED LDR
  }
  else{
    lightState = 0;               // TERANG
    digitalWrite(ledLight, LOW);          //untuk menyalakan LED LDR
  }

  // RAIN
  if(rainValue <= rainParameter1){                   
    rainState = 1;
    digitalWrite(ledRain, HIGH);
  }
  else{
    rainState = 0;
    digitalWrite(ledRain, LOW);
  }
}
void monitor(){
  currentPrintTime = millis();
  if(currentPrintTime - prevPrintTime >= 950){
  Serial.print("Temp: "); Serial.print(tempValue);Serial.print(" || ");
  Serial.print("PIR: "); Serial.print(pirValue);Serial.print(" || ");
  Serial.print("IR1: "); Serial.print(irRightValue);Serial.print(" || ");
  Serial.print("IR2: "); Serial.print(irLeftValue);Serial.print(" || ");
  Serial.print("Light: "); Serial.print(lightValue);Serial.print(" || ");
  Serial.print("Rain: "); Serial.print(rainValue);Serial.print(" || ");
  Serial.print("PEOPLE DETECTED: "); Serial.println(people_detected);
  Serial.print(conditionStateP);Serial.print(" || "); Serial.println(conditionStateL);
  prevPrintTime = currentPrintTime;
  }

  
  // Serial.print("IN COUNTER: "); Serial.print(in_counter);Serial.print(" || ");
  // Serial.print("OUT COUNTER: "); Serial.print(out_counter);Serial.println("");
}


void checkWalkIn(){
  if( ir_right_state != ir_right_state_last ){
    ir_right_state_last = ir_right_state;
    if( (bWalkIn == false) && ( ir_right_state == LOW ) ){
      bWalkIn = true;
      tm = millis();
    }
  }
  if( (millis() - tm) > DELAY_TIMEOUT ){
    bWalkIn = false;
  }
  if( bWalkIn && (ir_left_state == LOW) && (ir_right_state == HIGH) ){
    bWalkIn = false;
    in_counter++;
    Serial.print("in_counter: "); Serial.println(in_counter);
  }
}

void checkWalkOUT(){
  if( ir_left_state != ir_left_state_last ){
    ir_left_state_last = ir_left_state;
    if( (bWalkOut == false) && ( ir_left_state == LOW ) ){
      bWalkOut = true;
      tm = millis();
    }
  }
  if( (millis() - tm) > DELAY_TIMEOUT ){
    bWalkOut = false;
  }
  if( bWalkOut && (ir_right_state == LOW) && (ir_left_state == HIGH) ){
    bWalkOut = false;
    out_counter++;
    Serial.print("out_counter: ");Serial.println(out_counter);
  }
}

void peopleCounter(){
  checkWalkIn();
  checkWalkOUT();
  people_detected = in_counter - out_counter;
  if(people_detected < 0){
    people_detected = 0;
    out_counter = in_counter;
  }
  lcd.setCursor(7,0);
  lcd.print(people_detected);
  // if(people_detected >= 4){
  //   in_counter = 4;
  //   out_counter = 0;
  // lcd.setCursor(7,0);
  // lcd.print("F");
  // }
}

void condition(){
  if(people_detected > 0 && tempState == 1){
    digitalWrite(ledPayung, HIGH);
    conditionStateP  = 1;
    if(conditionStateP != cp_then){
      rev_now = motorRotation3;
      rev = rev_now - rev_then;
      lcd.setCursor(10,0);
      lcd.print("1");
      lcd.setCursor(14,0);
      lcd.print(rev_now);
      motorStepper(rev);
      cp_then = conditionStateP;
      rev_then = rev_now;
    }
  }
  
  else if(people_detected > 0 && rainValue <= rainParameter3){
    
    digitalWrite(ledPayung, 255);
    conditionStateP  = 2;
    if(conditionStateP != cp_then){
    rev_now = motorRotation3;
    rev = rev_now - rev_then;
    lcd.setCursor(10,0);
    lcd.print("2");
    lcd.setCursor(14,0);
    lcd.print(rev_now);
    motorStepper(rev);
    cp_then = conditionStateP;
    rev_then = rev_now;
    }
  }
  else if(people_detected > 0 && rainValue > rainParameter3 && rainValue <rainParameter2){
    digitalWrite(ledPayung, 180);
    conditionStateP  = 3;
      if(conditionStateP != cp_then){
      rev_now = motorRotation2;
      rev = rev_now - rev_then;
      lcd.setCursor(10,0);
      lcd.print("3");
      lcd.setCursor(14,0);
      lcd.print(rev_now);
      motorStepper(rev);
      cp_then = conditionStateP;
      rev_then = rev_now;
    }
    //conditionStringP = "ORANG + SEDANG";
    // Serial.print("ORANG + SEDANG");
  }
  else if(people_detected > 0 && rainValue >= rainParameter2 && rainValue <= rainParameter1){
    digitalWrite(ledPayung, 100);
    conditionStateP = 4;
      if(conditionStateP != cp_then){
      rev_now = motorRotation1;
      rev = rev_now - rev_then;
      lcd.setCursor(10,0);
      lcd.print("4");
      lcd.setCursor(14,0);
      lcd.print(rev_now);
      motorStepper(rev);
      cp_then = conditionStateP;
      rev_then = rev_now;
    }
  }
  else{
    digitalWrite(ledPayung, 0);
    conditionStateP = 5;
      if(conditionStateP != cp_then){
      rev_now = motorRotation0;
      rev = rev_now - rev_then; 
      lcd.setCursor(10,0);
      lcd.print("5"); 
      lcd.setCursor(14,0);
      lcd.print(rev_now);
      motorStepper(rev);
      cp_then = conditionStateP;
      rev_then = rev_now;
    }
    //motorStepper(rev);
  }

  if(people_detected > 0 && lightState == 1){
    digitalWrite(ledLamp, HIGH);
     conditionStateL = 1;
     //conditionStringL = "ORANG + GELAP";
    // Serial.print(" || "); Serial.println("ORANG + GELAP");
  }
  else{
    digitalWrite(ledLamp, LOW);
    conditionStateL = 2;
    //conditionStringL = "TAK ADA ORANG + TERANG";
    // Serial.print(" || "); Serial.println("TAK ADA ORANG + TERANG");
  }
}

void motorStepper(int rev){
  Serial.println("MOTOR STEPPER");
  if(cp_then != conditionStateP){
    //count = 3;
    while (rev > 0){
      Serial.print("revolution forward= "); Serial.println(rev);
      stepperName.step(stepsPerRevolution);
      rev--;
    }
    while (rev < 0){
      Serial.print("revolution backward = -"); Serial.print(rev);
      stepperName.step(-stepsPerRevolution);
      rev++;
    }
    motorFlag = 2;
  }
  else{
    stepperName.step(0);
  }
}

void lcdDisplay(){
  //Serial.println("LCD DISPLAY");
  switch(conditionStateP){
    case 1:
      lcd.setCursor(10,0);
      lcd.print("1");
      break;
    case 2:
      lcd.setCursor(10,0);
      lcd.print("2");
      break;
    case 3:
      lcd.setCursor(10,0);
      lcd.print("3");
      break;
    case 4:
      lcd.setCursor(10,0);
      lcd.print("4");
      break;
    case 5:
    // Serial.println("SWITHC CASE 5");
      lcd.setCursor(10,0);
      lcd.print("5");
      break;
  }
  switch(lightState){
    case 1:
      lcd.setCursor(12,0);
      lcd.print(conditionStateL);
      break;
    case 0:
      // Serial.println("SWITHC CASE 2");
      lcd.setCursor(12,0);
      lcd.print(conditionStateL);
    break;
  }
}


void loop() {
  sensorReading();
  monitor();
  stateAndLamp();
  peopleCounter();
  
  condition();
  lcdDisplay();
  //motorStepper(rev);
  delay(100);

}

