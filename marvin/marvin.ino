#include <ArduinoRobot.h>

#include <Wire.h>
#include <SPI.h>

#define DIST_TRIG_PIN TKD3
#define DIST_ECHO_PIN TKD1
#define DIST_FOEFEL_FACTOR 1.25

#define MAX_SPEED 160
#define MAX_SPEED_REVERSE 100

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
}

#define FLOOR 920
#define LINE 100
#define THRESHOLD (LINE + (2 *(FLOOR - LINE) / 3))

int lineDetected() {

  Robot.updateIR();

  int avg = 0;

  for(int i=0; i < 5; ++i) {
    avg += Robot.IRarray[i];
    Serial.print(Robot.IRarray[i]);
    Serial.print(", ");
  }

  avg = avg / 5;
  Serial.println("");
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


void loop() {

  // Your progam loop

  if(lineDetected() && lineDetected()) {
      backAndTurn();
      //Serial.println("line");
      Robot.beep(BEEP_SIMPLE);
  } 
  else {
    //Serial.println("noline");
      Robot.motorsWrite(MAX_SPEED, MAX_SPEED);
  }
  delay(10);
}




