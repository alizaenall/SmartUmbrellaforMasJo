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
#define irRightSensor  4
#define irLeftSensor  5
#define lightSensor A0
#define rainSensor A1
// LED PIN
#define ledTemp 7
#define ledPir 8
#define ledIr1 9
#define ledIr2 10
#define ledLight 11
#define ledRain 12

#define DELAYTIMEOUT 1500

// Sensor State Variabel
int tempState = 0;              //Mendeteksi suhu
int pirState = 0;               //Mendeteksi orang

int lightState = 0;             //Mendeteksi cahaya
int rainState = 0;              //Mendeteksi hujan

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
unsigned long tm;
//BIDIRECTIONAL Counting Variable
int count = 0;
int countState = 0;
int irRightState = 0;               //Mendeteksi obstacle 1 (kiri)
int irLeftState = 0;               //Mendeteksi obstacle 2 (kanan)
int irRightStateLast = -1;
int irLeftStateLast = -1;
int inCounter = 0;
int outCounter = 0;
bool bWalkIn = false;
bool bWalkOut = false;



void setup() {
  //Initiate the serial monitor
  Serial.begin(9600);
  // Initialize Sensor PIN
  //PIN DHT11 diinitialize dengan library   //PIN 2
  pinMode(pirSensor, INPUT);                //PIN 3
  pinMode(irRightSensor, INPUT);                //PIN 4
  pinMode(irLeftSensor, INPUT);                //PIN 5
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
  // IR1
  irRightValue = digitalRead(irRightSensor);
  // IR2
  irLeftValue = digitalRead(irLeftSensor);
  // LDR
  lightValue = analogRead(lightSensor);
  // RAIN
  rainValue = analogRead(rainSensor);    
  
  prevReadTime = currentReadTime;
  }
}

void stateAndLamp(){
  //DHT
  if(tempValue >= 30){
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
  Serial.print("IN COUNTER: "); Serial.print(inCounter);Serial.print(" || ");
  Serial.print("OUT COUNTER: "); Serial.print(outCounter);Serial.println("");
}

// void counting(){
//   Serial.println("COUNT FUNCTION");
//   if(irRightState == 1 && irLeftState == 0){
//     countState = 1;
//     Serial.println("IF 1");
//   }
//   else if(irRightState == 0 && irLeftState == 1 && countState == 1){
//     count++;
//     Serial.println("IF COUNTSTATE 1");
//     countState = 0;
//   }
//   // if(irRightState == 0 && irLeftState == 1){
//   //   countState = 2;
//   //   Serial.println("IF 2");
//   // }
//   // else if(irRightState == 1 && irLeftState == 0 && countState == 2){
//   //   count--;
//   //   Serial.println("IF COUNTSTATE 2");
//   // }


//   // currentCountTime = millis();
//   // if(irRightState == 1){
//   //   prevCountTime = CurrentCountTime;

//   //   if(currentCountTime - prevCountTime <= 1000){
//   //     if(irLeftState == 1){
//   //       count++;
//   //     }
//   //     else{
//   //       prevPrintTime = currentPrintTime
//   //     }
//   //   }
//   // else{
//   //   if((currentCountTime - PrevCountTime)>= 1000){
      
//   //   }
//   // }
//   // }
// }

void checkWalkIn(){
  if(irRightState != irRightStateLast){
    irRightStateLast = irRightState;
    if((bWalkIn == false) && (irRightState == 0)){
      bWalkIn = true;
      tm = millis();
    } 
  }
  if((millis() - tm) > DELAYTIMEOUT){
    bWalkIn = false;
  }
  if( bWalkIn && (irLeftState == 0) && (irRightState == 1) ){
    bWalkIn = false;
    inCounter++;
    Serial.print("in_counter");
  }
}

void checkWalkOut(){
  if(irLeftStateLast != irLeftStateLast){
    irLeftStateLast = irLeftState;
    if( (bWalkOut == false) && (irLeftState == LOW) ){
      bWalkOut = true;
      tm = millis();
    } 
  }
  if( (millis() - tm) > DELAYTIMEOUT){
    bWalkOut = false;
  }
  if( bWalkOut && (irRightState == 0) && (irLeftState == 1) ){
    bWalkOut = false;
    outCounter++;
  }
}

void condition(){

}


void loop() {
  sensorReading();
  stateAndLamp();
  //condition();
  currentPrintTime = millis();
  if(currentPrintTime - prevPrintTime >= 950){
    monitor();
    prevPrintTime = currentPrintTime;
  }
  checkWalkIn();
  checkWalkOut();
  delay(50);
}

