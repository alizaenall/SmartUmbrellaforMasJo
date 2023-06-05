/*
//Progress 22/05/23
baru 2 sensor, PIR dan DHT11
//Progress 28/05/23

*/

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h> 

//DHT 11 SENSOR
#define DHTPIN 2        // Digital pin connected to the DHT sensor 
#define DHTTYPE DHT11   //DHT11 sensor +-5% accuracy
//Make an object for DHT library
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// SENSOR PIN
#define pirSensor  3              //HIGH and LOW value of PIR sensor
// #define ir_right_pin  4
// #define ir_left_pin  5
#define lightSensor A0
#define rainSensor A1
// LED PIN
#define ledTemp 7
#define ledPir 8
#define ledIr1 9
#define ledIr2 10
#define ledLight 11
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

void setup() {
  //Initiate the serial monitor
  Serial.begin(9600);
  // Initialize Sensor PIN
  //PIN DHT11 diinitialize dengan library   //PIN 2
  pinMode(pirSensor, INPUT);                //PIN 3
  pinMode(ir_right_pin, INPUT);                //PIN 4
  pinMode(ir_left_pin, INPUT);                //PIN 5
  pinMode(lightSensor, INPUT);              //PIN A0
  pinMode(rainSensor, INPUT);               //PIN A1
  //LED INdicator
  pinMode(ledTemp, OUTPUT);
  pinMode(ledPir, OUTPUT);
  pinMode(ledIr1, OUTPUT);
  pinMode(ledIr2, OUTPUT);
  pinMode(ledLight, OUTPUT);
  pinMode(ledRain, OUTPUT);
  

  //Inititate DHT sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

void sensorReading(){
  // DHT11
  currentTempTime = millis();
  if(currentTempTime - prevTempTime >= 1000){
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    tempValue = event.temperature;     // Reading the environment temperature

    prevTempTime = currentTempTime;
  }
  currentReadTime = millis();
  if(currentReadTime - prevReadTime >= 50){
  // PIR
  pirValue = digitalRead(pirSensor); // Reading the presence of human infrared
  // // IR1
  irRightValue = digitalRead(ir_right_pin);
  // // IR2
  irLeftValue = digitalRead(ir_left_pin);
  // LDR
  lightValue = analogRead(lightSensor);
  // RAIN
  rainValue = analogRead(rainSensor);

  //FOR BIDIRECTIONAL ALGORITHM
  ir_right_state = digitalRead( ir_right_pin );
  ir_left_state =  digitalRead( ir_left_pin );    
  
  prevReadTime = currentReadTime;
  }
}

void stateAndLamp(){
  //DHT
  if(tempValue >= 32){
    tempState = 1;
    digitalWrite(ledTemp, HIGH);
  }
  else{
    tempState = 0;
    digitalWrite(ledTemp, LOW);
  }

  //PIR
  if(pirValue == 1){
    pirState = 0;
    digitalWrite(ledPir, HIGH);
  }
  else{
    pirState = 1;
    digitalWrite(ledPir, LOW);
  }

  //IR 1
  if(irRightValue == 0){
    irRightState = 0;
    digitalWrite(ledIr1, HIGH);
  }
  else{
    irRightState = 1;
    digitalWrite(ledIr1, LOW);
  }

  //IR 2
  if(irLeftValue == 0){
    irLeftState = 1;
    digitalWrite(ledIr2, HIGH);
  }
  else{
    irRightState = 0;
    digitalWrite(ledIr2, LOW);
  }

  // LDR
  if(lightValue >= 700){
    lightState = 1;
    digitalWrite(ledLight, HIGH);
  }
  else{
    lightState = 0;
    digitalWrite(ledLight, LOW);
  }

  // RAIN
  if(rainValue <= 700){
    rainState = 1;
    digitalWrite(ledRain, HIGH);
  }
  else{
    rainState = 0;
    digitalWrite(ledRain, LOW);
  }
}
void monitor(){
  Serial.print("Temp: "); Serial.print(tempValue);Serial.print(" || ");
  Serial.print("PIR: "); Serial.print(pirValue);Serial.print(" || ");
  Serial.print("IR1: "); Serial.print(irRightValue);Serial.print(" || ");
  Serial.print("IR2: "); Serial.print(irLeftValue);Serial.print(" || ");
  Serial.print("Light: "); Serial.print(lightValue);Serial.print(" || ");
  Serial.print("Rain: "); Serial.print(rainValue);Serial.print(" || ");
  Serial.print("PEOPLE DETECTED: "); Serial.println(people_detected);
  Serial.print(conditionStateP);Serial.print(" || "); Serial.println(conditionStateL);
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
  }
}

void condition(){
  if(people_detected > 0 && tempState == 1){
    digitalWrite(ledPayung, HIGH);
    conditionStateP  = 1;
    // Serial.print("ORANG + TEMP");
    // conditionStringP = "ORANG + TEMP";
  }
  else if(people_detected > 0 && rainValue <= 300){
    digitalWrite(ledPayung, 255);
    conditionStateP  = 2;
    //conditionStringP = "ORANG + DERAS";
    // Serial.print("ORANG + DERAS");
  }
  else if(people_detected > 0 && rainValue > 300 && rainValue <500){
    digitalWrite(ledPayung, 180);
    conditionStateP  = 3;
    //conditionStringP = "ORANG + SEDANG";
    // Serial.print("ORANG + SEDANG");
  }
  else if(people_detected > 0 && rainValue >= 500 && rainValue <=700){
    digitalWrite(ledPayung, 100);
    conditionStateP = 4;
    //conditionStringP = "ORANG + RINGAN";
    // Serial.print("ORANG + RINGAN");
  }
  else{
    digitalWrite(ledPayung, 0);
    conditionStateP = 5;
    //conditionStringP = "TAK ADA ORANG + TIDAK HUJAN / TIDAK PANAS";
    // Serial.print("TAK ADA ORANG + TIDAK HUJAN / TIDAK PANAS");
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


void loop() {
  sensorReading();
  stateAndLamp();
  currentPrintTime = millis();
  if(currentPrintTime - prevPrintTime >= 950){
    monitor();
    prevPrintTime = currentPrintTime;
  }
  peopleCounter();
  condition();
  delay(50);
}

