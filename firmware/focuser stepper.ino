/*Example sketch to control a stepper motor with DRV8825 stepper motor driver, AccelStepper library and Arduino: continuous rotation. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define enable 7
#define M0 4
#define M1 5
#define M2 6
#define motorInterfaceType 1

char version[] = "0.0.1";
int speed = 800;
long maxSteps = 50000;
int stepMode = 32;
char buffSend[15];
// Create a new instance of the AccelStepper class:
AccelStepper focuser = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  Serial.begin(9600);
  focuser.setMaxSpeed(12800);
  pinMode(M0, INPUT);
  digitalWrite(M0, HIGH);
  pinMode(M1, INPUT);
  digitalWrite(M1, LOW);
  pinMode(M2, INPUT);
  digitalWrite(M2, HIGH);
}

void focuserComm(){
  if (Serial.available() > 0) {
    String strInput = Serial.readString();
    focuserProtocol(strInput);
  }
}



void focuserProtocol(String strInput){
  // Set new position
  if (strInput.substring(1, 3) == "SN"){
    long ticks = strInput.substring(3, strInput.indexOf('#')).toFloat();
    focuser.moveTo(ticks);
    focuser.setSpeed(speed);
  }
  // Abort focuser
  if (strInput.substring(1, 3) == "FQ"){
    focuser.stop();
  }
  // Sync focuser
  if (strInput.substring(1, 3) == "SP"){
    long syncPosition = long(strInput.substring(3, strInput.indexOf('#')).toFloat());
    focuser.setCurrentPosition(syncPosition);
  }
  // Set speed
  if (strInput.substring(1, 3) == "SD"){
    speed = long(strInput.substring(3, strInput.indexOf('#')).toFloat());
  }
  // Set step mode.
  if (strInput.substring(1, 3) == "SM"){
    stepMode = strInput.substring(3, strInput.indexOf('#')).toFloat();
    if (stepMode == 1){
      digitalWrite(M0, LOW);
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
    }
    if (stepMode == 2){
      digitalWrite(M0, HIGH);
      digitalWrite(M1, LOW);
      digitalWrite(M2, LOW);
    }
    if (stepMode == 4){
      digitalWrite(M0, LOW);
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
    }
    if (stepMode == 8){
      digitalWrite(M0, HIGH);
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
    }
    if (stepMode == 16){
      digitalWrite(M0, LOW);
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
    }
    if (stepMode == 32){
      digitalWrite(M0, HIGH);
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
    }
  }
  // Get position.
  if (strInput.substring(1, 3) == "GP"){
    sprintf(buffSend, "%d#", focuser.currentPosition());
    Serial.println(buffSend);
  }
  // Get stepMode.
  if (strInput.substring(1, 3) == "GH"){
    sprintf(buffSend, "%d#", stepMode);
    Serial.println(buffSend);
  }
  // Get speed.
  if (strInput.substring(1, 3) == "GD"){
    sprintf(buffSend, "%d#", speed);
    Serial.println(buffSend);
  }
  // Get version.
  if (strInput.substring(1, 3) == "GV"){
    sprintf(buffSend, "%s#", version);
    Serial.println(buffSend);
  }
  // Get temperature.
  if (strInput.substring(1, 3) == "GT"){
    float temperature = 20.0;
    sprintf(buffSend, "%lf#", temperature);
    Serial.println(buffSend);
  }
  // Is focuser moving ?
  if (strInput.substring(1, 3) == "GI"){
    int isMoving;
    if (focuser.isRunning() == true){
      isMoving = 1;
    }
    else if (focuser.isRunning() == false){
      isMoving = 0;
    }
    sprintf(buffSend, "%d#", isMoving);
    Serial.println(buffSend);
  }
}

void loop() {
  focuserComm();
  if (focuser.targetPosition() == focuser.currentPosition()){
    focuser.setSpeed(0);
  }
  focuser.runSpeedToPosition();
}
