//Modular Code 
//Bidirectional Counter with 2 IR Sensor

#define inIR 7
#define outIR 8
#define relay 6
int inIRCount = 0;
int outIRCount = 0;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(inIR, INPUT_PULLUP);
  pinMode(outIR, INPUT_PULLUP);
}

void loop() {
  inIRCount = digitalRead (inIR);
  outIRCount = digitalRead (outIR);

  if(inIRCount == LOW){
    count++;
    delay(500);
  } 
  if(outIRCount == LOW){
    count--;
    delay(500);
  }

  if(count<=0){
    digitalWrite(relay, LOW);
  }
  // put your main code here, to run repeatedly:
  if(digitalRead(inIR)){
    count++;
  }
  
  Serial.println("Visitor =  "); Serial.println(count);
}
