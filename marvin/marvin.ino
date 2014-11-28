#include <ArduinoRobot.h>

#include <Wire.h>
#include <SPI.h>

#include <IRremote.h> //IR Remote uses TKD4 and configures this pin itself

#define DIST_TRIG_PIN TKD3
#define DIST_ECHO_PIN TKD1
#define DIST_FOEFEL_FACTOR 1.25

#define MAX_SPEED 160
#define MAX_SPEED_REVERSE 100

// IR stuff
const int IR_RECV_REAR = TKD0;
const int ULTRASONE_RX = TKD1;
const int IR_RECV_FRONT = TKD2;
const int ULTRASONE_TX =TKD3;
const int LASER_PIN = TKD5;
 
IRsend irsend;
IRrecv irrecv_front(IR_RECV_FRONT);
//IRrecv irrecv_rear(IR_RECV_REAR);
decode_results irrecv_results;
long distance = 0;

unsigned long scanEnd;

void pointTo(int angle) {
  int target=angle;
  uint8_t speed=80;
  target=target%360;
  if(target<0){
    target+=360;
  }
  int direction=angle;
  while(1){
    int currentAngle=Robot.compassRead();
    int diff=target-currentAngle;
    direction=180-(diff+360)%360;
    if(direction<0){
      Robot.motorsWrite(speed,-speed);//right
      delay(10);
    }else{
      Robot.motorsWrite(-speed,speed);//left
      delay(10);
    }
    if(abs(diff)<5){
      Robot.motorsStop();
      return;
    }
  }
}

void turn(int angle) {
  Robot.motorsStop();
  int currentAngle = Robot.compassRead();
  int target = currentAngle + angle;
  pointTo(target);
}

void setup() {

  Robot.begin();

  // Run setup code

  Serial.begin(9600);

  Robot.beginSpeaker();

  pinMode(DIST_TRIG_PIN, OUTPUT);
  pinMode(DIST_ECHO_PIN, INPUT);
  
  scanEnd = millis() + 200;
  irrecv_front.enableIRIn(); // Start the receiver
}

#define FLOOR 920
#define LINE 100
#define THRESHOLD (LINE + (2 *(FLOOR - LINE) / 3))

int lineDetected() {

  Robot.updateIR();

  int avg = 0;

  for(int i=0; i < 5; ++i) {
    avg += Robot.IRarray[i];
  }

  avg = avg / 5;
  if(avg < THRESHOLD) {
    return 1;
  } 
  else {
    return 0;
  }
}

void drive(int left, int right)
{
    Robot.motorsStop();
    delay(5);
    Robot.motorsWrite(left, right);
}
  

void backAndTurn(){

    drive(-MAX_SPEED,-MAX_SPEED);
    // 2 seconds reverse driving
    delay(2000);
    turn(-85);
    // wait because of possible wobble?
    delay(500);
}

long distanceInCm() {
  digitalWrite(DIST_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(DIST_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DIST_TRIG_PIN, LOW);
  long duration = pulseIn(DIST_ECHO_PIN, HIGH);
  return (duration) / 29.1 / DIST_FOEFEL_FACTOR;
}

int shootAndWait()
{
  unsigned long endTime = millis() + 2000;
  unsigned long time;

  Robot.beep(BEEP_DOUBLE);
  digitalWrite(LASER_PIN, HIGH);
  while(millis() < endTime)
  {
    irsend.sendNEC(0xdeaddead, 32); // 120ms
    scanIR(); // min 200ms
  }
  digitalWrite(LASER_PIN, LOW);
  Robot.beep(BEEP_DOUBLE);
 
  endTime = millis() + 3000;
  while(millis() < endTime)
  {
    scanIR(); // min 200ms
  }
  Robot.beep(BEEP_SIMPLE);
}

int scanIR()
{
  irrecv_front.enableIRIn(); // Start the receiver

  delay(200);
  if(irrecv_front.decode(&irrecv_results)) {
    Robot.beep(BEEP_LONG);
  }
}

void scanNoWait()
{
  if(irrecv_front.decode(&irrecv_results)) {
    Robot.beep(BEEP_LONG);
  }
  
  irrecv_front.enableIRIn(); // Start the receiver
}

void loop() {

  // Your progam loop
  //Serial.println("Loop started");
  if(lineDetected() && lineDetected()) {
      backAndTurn();
      //Serial.println("line");
      Robot.beep(BEEP_SIMPLE);
  } 
  else {
    //Serial.println("noline");
      Robot.motorsWrite(MAX_SPEED, MAX_SPEED);
  }
  
/*  distance = distanceInCm();
  //Serial.println(distance);
  if(distance != 0 && distance < 20)
  { 
    Robot.beep(BEEP_LONG);
  }*/
  if(millis() > scanEnd)
  {
    //scanNoWait();
    scanEnd = millis() + 200;
  }
  
  delay(10);
}




