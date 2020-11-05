#include <MeMCore.h>
#include "Notes.h"

/*** Configurations ***/
#define IR_LEFT         A1
#define IR_RIGHT        A0
#define LDR_PIN         A6
#define LED_PIN         7
#define BUZZER_PIN      8
MeDCMotor               leftWheel(M1);
MeDCMotor               rightWheel(M2);
MeLineFollower          lineFinder(PORT_1);     
MeUltrasonicSensor      ultraSensor(PORT_3);    
MeRGBLed                led(LED_PIN);
MeBuzzer                buzzer;

// Movement
#define MOTOR_SPEED                220
#define TIME_TURN_90_DEGREE        260
#define TIME_GRID                  35                       // time to travel 1 grid
#define TIME_DELAY                 20                       // delay b4 recheck position
#define TIME_MUL                   5                        // time multiplier for IR adjustment
#define K_DIST                     (255/2)                  // max correction to mvmt
#define D_FRONT                    5                        // cm
#define V_LEFTIR                   435                      // threshold for IR sensor values
#define V_RIGHTIR                  570
#define RIGHT_TURN                 556

// Color
#define COL_DIST        4000                    // threshold for comparing colours
#define BLA_VAL         {270, 255, 284}
#define GRE_VAL         {323, 282, 325}

#define RED_ARR         {204, 81, 77}           // normalised RGB values
#define GRE_ARR         {52, 124, 66}
#define YEL_ARR         {255, 206, 121}
#define PUR_ARR         {140, 145, 189}
#define BLU_ARR         {167, 233, 220}
#define BLA_ARR         {0,0,0}
#define NUM_OF_COLOURS  6                       // black, red, green, yellow, purple, blue

// Calibration
#define CALLIBRATE_SEC  3                       // delay before IR and colour calibration
#define NUM_OF_SAMPLES  50                      // number of samples for calibration
#define IR_WAIT         100                     // delay between IR measurements
#define RGB_WAIT        50                      // delay between each LED flash
#define LDR_WAIT        10                      // delay between each LDR measurement
#define LED_MAX         255                     // max value of each RGB component



/********** Global Variables **********/
bool busy = false;

int irArray[2][2] = {{68,10},{81,19}}; // left-right, minmax
int blackArray[] = BLA_VAL;
int greyDiff[] = GRE_VAL;
static int allColourArray[6][3] = {BLA_ARR, RED_ARR, GRE_ARR, YEL_ARR, PUR_ARR, BLU_ARR};
  


/********** Main Program **********/
void setup() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(9600);

//   calibrateWB();
//   calibrateIR();
//  uTurn();
//  doubleRight();
//  doubleLeft();
//  getColourCode();
//  printColour(getColour());
  busy = false;
//  getAvgDist();
}

void stopRunning(const int i);

void loop() {
  if (busy) return;

  if (isAtBlackLine() == true) { // both sensors not in black line
    // waypoint detected
    busy = true;
    stopRunning(0);
  
    // color challenge
    int colourRes;
    do {
      colourRes = getColour();
//      printColour(colourRes);
    } while (colourRes == -1);
  
    if (colourRes > 0) { // is color challenge (not black)
      colorWaypoint(colourRes);
      busy = false;
      return;
    }
    
    // finished
    finishWaypoint();
  }
  else {
    moveForward();
  }
}


/*** Movement ***/
void stopRunning(const int i = 10) {
  rightWheel.stop();
  leftWheel.stop();
  if (i) delay(TIME_DELAY * i);
}

// right motor is 
int rightSpeed = MOTOR_SPEED * 0.78;
int leftSpeed = -MOTOR_SPEED;

void moveForward() {
//  Serial.println(ultraSensor.distanceCm());
  if (ultraSensor.distanceCm() < D_FRONT) {
    stopRunning();
    return;
  }
  
  int new_delay = 50;
  int leftReading = analogRead(IR_LEFT);
  int rightReading = analogRead(IR_RIGHT);

  if (leftReading < V_LEFTIR) {
    rightWheel.stop();
    leftWheel.run(-MOTOR_SPEED);
    delay(new_delay);
  }
  else if (rightReading < V_RIGHTIR) {
    leftWheel.stop();
    rightWheel.run(MOTOR_SPEED);
    delay(new_delay);
  }
  else {
    leftWheel.run(leftSpeed);
    rightWheel.run(rightSpeed);
  }
}

void forwardGrid() {
  for (int i = 0; i < TIME_GRID; i++) { 
    if (ultraSensor.distanceCm() < D_FRONT) break;
    leftWheel.run(leftSpeed);
    rightWheel.run(rightSpeed);
    delay(TIME_DELAY);
  }
  stopRunning();
}

void turnRight() {
  leftWheel.run(-MOTOR_SPEED);
  rightWheel.run(-MOTOR_SPEED);
  delay(TIME_TURN_90_DEGREE);
  stopRunning();
}

void turnLeft() {
  leftWheel.run(MOTOR_SPEED);
  rightWheel.run(MOTOR_SPEED);
  delay(TIME_TURN_90_DEGREE);
  stopRunning();
}

void doubleRight() {
  turnRight();
  stopRunning(15);
  forwardGrid();
  stopRunning(15);
  turnRight();
}

void doubleLeft() {
  turnLeft();
  stopRunning(15);
  forwardGrid();
  stopRunning(15);
  turnLeft();
}

void uTurn() {
  int rightReading = analogRead(IR_RIGHT);

  if (rightReading < RIGHT_TURN) {
    turnLeft();
    turnLeft();
  }
  else {
    turnRight();
    turnRight();
  }
}


/*** Sensors ***/
long long square(const long long x) { return x * x; }

int getColour() {
  // Read colours
  float colourArray[3] = {0};
  for (int i = 0; i < 3; i++) { // red, green, blue
    led.setColor( // one-hot encoding
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    );
    led.show();
    delay(RGB_WAIT);

    for (int j = 0; j < NUM_OF_SAMPLES; j++) { // take avg reading
      colourArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    
    colourArray[i] /= NUM_OF_SAMPLES;
    colourArray[i] = (colourArray[i] - blackArray[i]) * 255 / greyDiff[i];
    Serial.println(colourArray[i]);
  }
  led.setColor(0,0,0); led.show();

  // Find colour with min euclidean distance > COL_DIST
  int idx = -1;
  int min_dist = COL_DIST;
  for (int i = 0; i < 6; i++) {
    long long curr_dist = 0;
    for (int j = 0; j < 3; j++)
      curr_dist += square(allColourArray[i][j] - colourArray[j]);

    if (curr_dist <= COL_DIST && curr_dist < min_dist) {
      idx = i;
      min_dist = curr_dist;
    }
  }
  return idx;
  // Returns index of best color
  return -1;
}

bool isAtBlackLine() {
  return lineFinder.readSensors() == S1_IN_S2_IN;
}



/*** Waypoints ***/
void colorWaypoint(const int colourRes) {
  switch (colourRes) {
    case 1: turnLeft(); break;    // red
    case 2: turnRight(); break;   // green
    case 3: uTurn(); break;       // yellow
    case 4: doubleLeft(); break;  // purple
    case 5: doubleRight(); break; // light blue
  }
}

void finishWaypoint() {
  // keys and durations found in NOTES.h
  int wholenote = (60000 * 4) / tempo;
  for (int i = 0; i < notes * 2; i = i + 2) {
    // calculates the duration of each note
    int divider = melody[i + 1];
    int noteDuration = 0;
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }
  
    // we only play the note for 90% of the duration, leaving 10% as a pause
    buzzer.tone(BUZZER_PIN, melody[i], noteDuration * 0.9);
  
    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
  
    // stop the waveform generation before the next note.
    buzzer.noTone(BUZZER_PIN);
  }
  busy = 1;
}


/*** Calibration ***/
void calibrateIR() {
  Serial.println("CALIBRATING IR SENSORS");

  // Min values
  Serial.print("COVER SENSORS. Calibrating MIN in ");
  for (int i = CALLIBRATE_SEC; i > 0; --i) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  irArray[0][0] = irArray[1][0] = 0;
  for (int i = 0; i < NUM_OF_SAMPLES; ++i) {
    irArray[0][0] += analogRead(IR_LEFT);
    irArray[1][0] += analogRead(IR_RIGHT);
    delay(IR_WAIT);
  }
  irArray[0][0] /= NUM_OF_SAMPLES;
  irArray[1][0] /= NUM_OF_SAMPLES;
  Serial.println("done.");

  // Max values
  Serial.print("UNCOVER SENSORS. Calibrating MAX in ");
  for (int i = CALLIBRATE_SEC; i > 0; i--) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  irArray[0][1] = irArray[1][1] = 0;
  for (int i = 0; i < NUM_OF_SAMPLES; ++i) {
    irArray[0][1] += analogRead(IR_LEFT);
    irArray[1][1] += analogRead(IR_RIGHT);
    delay(IR_WAIT);
  }
  irArray[0][1] /= NUM_OF_SAMPLES;
  irArray[1][1] /= NUM_OF_SAMPLES;

  // Save range
  irArray[0][1] -= irArray[0][0]; // left range
  irArray[1][1] -= irArray[1][0]; // right range

  // Output calibrated
  Serial.print("int irArray[2][2] = {{");
  Serial.print(irArray[0][0]); Serial.print(",");
  Serial.print(irArray[0][1]); Serial.print("},{");
  Serial.print(irArray[1][0]); Serial.print(",");
  Serial.print(irArray[1][1]); Serial.println("}};");
}

void calibrateWB() {
  int whiteArray[3] = {0};

  // Max values
  Serial.print("Put WHITE sample. Calibrating MAX in ");
  for (int i = CALLIBRATE_SEC; i > 0; i--) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; i++) {
    led.setColor( // one-hot encoding
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    ); led.show();
    delay(RGB_WAIT);

    for (int j = 0; j < NUM_OF_SAMPLES; j++) {
      whiteArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    whiteArray[i] /= NUM_OF_SAMPLES;
  }
  led.setColor(0,0,0); led.show();
  Serial.println("done.");

  // Min values
  Serial.print("Put BLACK sample. Calibrating MIN in ");
  for (int i = CALLIBRATE_SEC; i > 0; i--) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; i++) {
    led.setColor( // one-hot encoding
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    ); led.show();
    delay(RGB_WAIT);
    
    blackArray[i] = 0;
    for (int j = 0; j < NUM_OF_SAMPLES; j++) {
      blackArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    blackArray[i] /= NUM_OF_SAMPLES;
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

void getColourCode() {
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

    for (int j = 0; j < NUM_OF_SAMPLES; j++) { // take avg reading
      colourArray[i] += analogRead(LDR_PIN);
      delay(LDR_WAIT);
    }
    colourArray[i] /= NUM_OF_SAMPLES;
    colourArray[i] = (colourArray[i] - blackArray[i]) * 255 / greyDiff[i];
    Serial.print(colourArray[i]); Serial.print(", ");
  }
  led.setColor(0,0,0); led.show();  
}

/* IR Sensor Calibration */
void getAvgDist() {
  // Take raw value and threshold
  float totalLeft = 0;
  float totalRight = 0;
  for (int i = 0; i < NUM_OF_SAMPLES; i++) {
    totalLeft += analogRead(IR_LEFT);
    totalRight += analogRead(IR_RIGHT);
  }
  float avgLeft = totalLeft / NUM_OF_SAMPLES;
  float avgRight = totalRight / NUM_OF_SAMPLES;
  Serial.print("LEFT: "); Serial.print(avgLeft); Serial.println();
  Serial.print("RIGHT: "); Serial.print(avgRight); Serial.println();
}
