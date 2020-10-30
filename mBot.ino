#include "Arduino.h"
#include "MeMCore.h"

#define FORWARD_SPEED           220
#define REVERSE_SPEED           FORWARD_SPEED
#define LEFT_FORWARD_SPEED      -FORWARD_SPEED
#define LEFT_REVERSE_SPEED      REVERSE_SPEED
#define RIGHT_FORWARD_SPEED     FORWARD_SPEED
#define RIGHT_REVERSE_SPEED     -REVERSE_SPEED

#define TIME_TURN_90_DEGREE 260
#define TIME_DELAY          20  // delay before recheck position
#define TIME_GRID           35
#define CALLIBRATE_SEC      3   

#define BLA_VAL         {260, 239, 265}
#define GRE_VAL         {289, 244, 294}

#define LDR_PIN         A6
#define IR_LEFT         A0
#define IR_RIGHT        A1
#define COLOUR_NO       50
#define K_DIST          128

#define DIST_LEFT          200
#define DIST_RIGHT         200

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
}

void loop() {
  moveForward();
}

void stopRunning(int i = 0) {
  leftMotor.stop();
  rightMotor.stop();
  if (i) {
    delay(TIME_DELAY * i);
  }
}

void moveForward() {
  int dx = getDist();
    
  // Normalise to MOTORSPEED
  int maxx = FORWARD_SPEED + (dx >= 0 ? dx : -dx);
  leftMotor.run((long long)(-FORWARD_SPEED + dx) * FORWARD_SPEED / maxx);
  rightMotor.run((long long)(FORWARD_SPEED + dx) * FORWARD_SPEED / maxx);
    
  delay(TIME_DELAY * 5);
  stopRunning();  
//  leftMotor.run(LEFT_FORWARD_SPEED);
//  rightMotor.run(RIGHT_FORWARD_SPEED);
//  delay(1000);
//  stopRunning();
}

int getDist() {
  // Take raw value and threshold
  int irVolt = analogRead(IR_LEFT);
  if (irVolt < DIST_LEFT) // turn right
    return (irVolt - DIST_LEFT) * K_DIST / DIST_LEFT;

  irVolt = analogRead(IR_RIGHT);
  if (irVolt < DIST_RIGHT) // turn left
    return (DIST_RIGHT - irVolt) * K_DIST / DIST_RIGHT;
  
  return 0;
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

void doubleRight() {
  turnRight();
  stopRunning(10);
  forwardGrid();
  stopRunning(10);
  turnRight();
  stopRunning(10);
  stopRunning();  
}

void uTurn() {
  turnRight();
  stopRunning();
  turnRight();
  stopRunning();  
}

bool isAtBlackLine() {
  return lineFinder.readSensors() == S1_IN_S2_IN;
}

void getColor() {
  // Read colours
    Serial.print("Put COLOR sample. Calibrating MIN in ");
  for (int i = 0; i < CALLIBRATE_SEC; i++) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  Serial.println();
  float colourArray[3] = {0};
  for (int i = 0; i < 3; ++i) { // red, green, blue
    led.setColor( // one-hot encoding
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    );
    led.show();
    delay(RGB_WAIT);

    for (int j = 0; j < COLOUR_NO; ++j) { // take avg reading
      colourArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    colourArray[i] /= COLOUR_NO;
    colourArray[i] = (colourArray[i] - blackArray[i]) * 255 / greyDiff[i];
    Serial.print(colourArray[i]);
    Serial.print(", ");
  }
  led.setColor(0,0,0); led.show();  
}

// Function for IR sensor
void calibrateIr() {
  int totalLeftDist = 0;
  int totalRightDist = 0;
  for (int i = 0; i < 50; i++) {
    totalLeftDist += analogRead(IR_LEFT);
  }
  for (int i = 0; i < 50; i++) {
    totalRightDist += analogRead(IR_RIGHT);
  }
  Serial.print("#DEFINE DIST_LEFT     ");Serial.println(totalLeftDist / 50);
  Serial.print("#DEFINE DIST_RIGHT    ");Serial.println(totalRightDist / 50);
}

// Function for calibrating color
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
