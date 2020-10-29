#include "Arduino.h"
#include "MeMCore.h"

#define FORWARD_SPEED           220
#define REVERSE_SPEED           220
#define LEFT_FORWARD_SPEED      -FORWARD_SPEED
#define LEFT_REVERSE_SPEED      REVERSE_SPEED
#define RIGHT_FORWARD_SPEED     FORWARD_SPEED
#define RIGHT_REVERSE_SPEED     -REVERSE_SPEED

#define TIME_TURN_90_DEGREE 350
#define TIME_DELAY          20  // delay before recheck position
#define TIME_GRID           45
#define CALLIBRATE_SEC      3   

#define BLA_VAL         {255, 217, 243}
#define GRE_VAL         {116, 108, 130}

#define LDR_PIN         A6
#define COLOUR_NO       50

#define LED_MAX         255
#define RGB_WAIT        100
#define LDR_WAIT        10

MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
MeRGBLed                led(7);
MeLineFollower lineFinder(PORT_2);

int blackArray[] = BLA_VAL;
int greyDiff[] = GRE_VAL;

void setup() {
  Serial.begin(9600);
  
  calibrateWB();
}

void loop() {
}

void stopRunning(int i = 0) {
  leftMotor.stop();
  rightMotor.stop();
  if (i) {
    delay(TIME_DELAY * i);
  }
}

void moveForward() {
  leftMotor.run(LEFT_FORWARD_SPEED);
  rightMotor.run(RIGHT_FORWARD_SPEED);
  delay(1000);
  stopRunning();
}

void turnLeft() {
  leftMotor.run(LEFT_REVERSE_SPEED);
  rightMotor.run(RIGHT_FORWARD_SPEED);  
  delay(TIME_TURN_90_DEGREE);
  stopRunning();
}

void turnRight() {
  rightMotor.run(RIGHT_REVERSE_SPEED);
  leftMotor.run(LEFT_FORWARD_SPEED);  
  delay(TIME_TURN_90_DEGREE);
  stopRunning();
}

void forwardGrid() {
  for (int i = 0; i < TIME_GRID; ++i) { 
    leftMotor.run(LEFT_FORWARD_SPEED);
    rightMotor.run(RIGHT_FORWARD_SPEED);
    delay(TIME_DELAY);
  }
  stopRunning();  
}

void doubleLeft() {
  turnLeft();
  stopRunning(10);
  forwardGrid();
  stopRunning(10);
  turnLeft();
  stopRunning(10);
  stopRunning();  
}

bool isAtBlackLine() {
  return lineFinder.readSensors() == S1_IN_S2_IN;
}

void calibrateWB() {
  Serial.println("===== CALIBRATING COLOR SENSORS (TOP) =====");
  int whiteArray[3] = {0};

  // Max values
  Serial.print("Put WHITE sample. Calibrating MAX in ");
  for (int i = 0; i < CALLIBRATE_SEC; i++) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; ++i) {
    led.setColor(
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    ); led.show();
    delay(RGB_WAIT);

    for (int j = 0; j < COLOUR_NO; ++j) {
      whiteArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    whiteArray[i] /= COLOUR_NO;
  }
  led.setColor(0,0,0); led.show();
  Serial.println("done.");

  // Min values
  Serial.print("Put BLACK sample. Calibrating MIN in ");
  for (int i = 0; i < CALLIBRATE_SEC; i++) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; ++i) {
    led.setColor(
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    ); led.show();
    delay(RGB_WAIT);
    
    blackArray[i] = 0;
    for (int j = 0; j < COLOUR_NO; j++) {
      blackArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    blackArray[i] /= COLOUR_NO;
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }
  led.setColor(0,0,0); led.show();
  Serial.println("done.");

  // Output calibrated
  Serial.print("#define BLA_VAL         {");
  Serial.print(blackArray[0]); Serial.print(", ");
  Serial.print(blackArray[1]); Serial.print(", ");
  Serial.print(blackArray[2]); Serial.println("}");
  Serial.print("#define GRE_VAL         {");
  Serial.print(greyDiff[0]); Serial.print(", ");
  Serial.print(greyDiff[1]); Serial.print(", ");
  Serial.print(greyDiff[2]); Serial.println("}");
}
