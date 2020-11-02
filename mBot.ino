#include <Wire.h>
#include <MeMCore.h>
#include <MeRGBLed.h>
#include "Notes.h"

/********** Settings **********/
#define LEFTIR_PIN      A0
#define RIGHTIR_PIN     A1
#define LDR_PIN         A6
#define LED_PIN         7
#define MUSIC_PIN       8
MeDCMotor               leftWheel(M1);          // -255 to 255
MeDCMotor               rightWheel(M2);         // 255 to -255, ie reversed
MeLineFollower          lineFinder(PORT_2);     
MeUltrasonicSensor      ultraSensor(PORT_3);    
MeRGBLed                led(LED_PIN);
MeBuzzer                buzzer;

// Movement
#define MOTORSPEED      220
#define TIMETURN        (61600/MOTORSPEED)      // time for 90deg turn
#define TIMEGRID        (180000/MOTORSPEED)     // time to travel 1 grid
#define TIMEDELAY       20                      // delay b4 recheck position
#define DELAYGRID       (TIMEGRID / TIMEDELAY)
#define TIMEMUL         5                       // time multiplier for IR adjustment
#define K_DIST          (255/2)                 // max correction to mvmt
#define D_FRONT         10                      // cm
#define V_LEFTIR        200                     // threshold for IR sensor values
#define V_RIGHTIR       200

// Color
#define COL_DIST        5000                    // 10000
#define BLA_VAL         {255, 217, 243}
#define GRE_VAL         {116, 108, 130}

#define RED_ARR         {185,35,35}             // normalised rgb vals
#define GRE_ARR         {45, 100, 60}
#define YEL_ARR         {255, 175, 100}         //325,230,135
#define PUR_ARR         {155,150,200}
#define BLU_ARR         {175,240,240}
#define BLA_ARR         {0,0,0}
#define NUMCOL          6                       // black, red, green, yellow, purple, blue

// Calibration
#define CALLIBRATE_SEC  3                       // delay b4 calibration
#define COLOUR_NO       50                      // 50
#define IR_WAIT         100                     // delay btw measurements. IMPT!
#define RGB_WAIT        100                     // 200
#define LDR_WAIT        10
#define LED_MAX         255
#define IR_MAX          1023



/********** Global Variables **********/
bool busy = true;

int irArray[2][2] = {{68,10},{81,19}}; // left-right, minmax
int blackArray[] = BLA_VAL;
int greyDiff[] = GRE_VAL;
static int allColourArray[6][3] = {BLA_ARR,RED_ARR,GRE_ARR,YEL_ARR,PUR_ARR,BLU_ARR};
  


/********** Main Program **********/
void setup() {
  pinMode(LEFTIR_PIN, INPUT);
  pinMode(RIGHTIR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MUSIC_PIN, OUTPUT);
  Serial.begin(9600);

  // calibrateWB();
  // calibrateIR();
  busy = false;
}

void stopMove(const int i);
void loop() {
  if (busy) return;

  if (lineFinder.readSensors() != S1_IN_S2_IN) { // both sensors not in black line
    moveForward();
    return;
  }
  
  // Waypoint detected!
  busy = true;
  stopMove(0);

  // Color Challenge
  int colourRes;
  do {
    colourRes = getColour();
    printColour(colourRes);
  } while (colourRes == -1);
  if (colourRes > 0) { // is color challenge (not black)
    colorWaypoint(colourRes);
    busy = false;
    return;
  }

  // Finished!
  finishWaypoint();
}


/********** Movement **********/
void stopMove(const int i = 10) {
  rightWheel.stop();
  leftWheel.stop();
  if (i) delay(TIMEDELAY * i);
}

void moveForward() {
  if (ultraSensor.distanceCm() < D_FRONT) return;
  const int dx = getDist();
    
  // Normalise to MOTORSPEED
  int maxx = MOTORSPEED + (dx >= 0 ? dx : -dx);
  leftWheel.run((long long)(-MOTORSPEED + dx) * MOTORSPEED / maxx);
  rightWheel.run((long long)(MOTORSPEED + dx) * MOTORSPEED / maxx);
    
  delay(TIMEDELAY * TIMEMUL);
  stopMove(0);
}

void forwardGrid() {
  for (int i = 0; i < DELAYGRID; ++i) { 
    if (ultraSensor.distanceCm() < D_FRONT) break;
    leftWheel.run(-MOTORSPEED);
    rightWheel.run(MOTORSPEED);
    delay(TIMEDELAY);
  }
  stopMove();
}

void turnRight() {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(-MOTORSPEED);
  delay(TIMETURN);
  stopMove();
}

void turnLeft() {
  leftWheel.run(MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(TIMETURN);
  stopMove();
}

void doubleRight() {
  turnRight();
  forwardGrid();
  turnRight();
}

void doubleLeft() {
  turnLeft();
  forwardGrid();
  turnLeft();
}

void uTurn() {
  turnRight();
  turnRight();
}



/********** Sensors **********/
int getDist() {
  // Take raw value and threshold
  int irVolt = analogRead(LEFTIR_PIN);
  if (irVolt < V_LEFTIR) // turn right
    return (irVolt - V_LEFTIR) * K_DIST / V_LEFTIR;

  irVolt = analogRead(RIGHTIR_PIN);
  if (irVolt < V_RIGHTIR) // turn left
    return (V_RIGHTIR - irVolt) * K_DIST / V_RIGHTIR;
  
  return 0;
}

// Color Sensor: Return nearest colour
long long square(const long long x) { return x * x; }
int getColour() {
  // Read colours
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
    Serial.println(colourArray[i]);
  }
  led.setColor(0,0,0); led.show();

  // Find colour with min euclidean distance > COL_DIST
  int idx = -1;
  int min_dist = COL_DIST;
  for (int i = 0; i < 6; ++i) {
    long long curr_dist = 0;
    for (int j = 0; j < 3; ++j)
      curr_dist += square(allColourArray[i][j] - colourArray[j]);

    if (min_dist > curr_dist && curr_dist > 0) {
      idx = i;
      min_dist = curr_dist;
    }
  }

  // Returns index of best color
  return idx;
}



/********** Waypoints **********/
void colorWaypoint(const int colourRes) {
  // red : left
  // green : right
  // yellow : 180deg within grid
  // purple : 2 left
  // light blue : 2 right
  switch (colourRes) {
    case 1: turnLeft(); break;
    case 2: turnRight(); break;
    case 3: uTurn(); break;
    case 4: doubleLeft(); break;
    case 5: doubleRight(); break;
  }
}

void finishWaypoint() {
  // keys and durations found in NOTES.h
  int wholenote = (60000 * 4) / tempo;
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    // calculates the duration of each note
    int divider = melody[thisNote + 1];
    int noteDuration = 0;
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }
  
    // we only play the note for 90% of the duration, leaving 10% as a pause
    buzzer.tone(MUSIC_PIN, melody[thisNote], noteDuration * 0.9);
  
    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
  
    // stop the waveform generation before the next note.
    buzzer.noTone(MUSIC_PIN);
  }
}



/********** Calibration **********/
// Used only for testing purposes, not for actual run.
int norm(const int val, const int *low_mult) {
  int tmp = 1023LL * (val - *low_mult) / *(low_mult+1);
  if (tmp < 0) tmp = 1; // so we know it's not 0
  else if (tmp > 1023) tmp = 1023;
  
  return tmp;
}

void calibrateIR() {
  Serial.println("===== CALIBRATING IR SENSORS (SIDE) =====");

  // Min values
  Serial.print("COVER SENSORS. Calibrating MIN in ");
  for (int i = CALLIBRATE_SEC; i > 0; --i) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  irArray[0][0] = irArray[1][0] = 0;
  for (int i = 0; i < COLOUR_NO; ++i) {
    irArray[0][0] += analogRead(LEFTIR_PIN);
    irArray[1][0] += analogRead(RIGHTIR_PIN);
    delay(IR_WAIT);
  }
  irArray[0][0] /= COLOUR_NO;
  irArray[1][0] /= COLOUR_NO;
  Serial.println("done.");

  // Max values
  Serial.print("UNCOVER SENSORS. Calibrating MAX in ");
  for (int i = CALLIBRATE_SEC; i > 0; --i) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  irArray[0][1] = irArray[1][1] = 0;
  for (int i = 0; i < COLOUR_NO; ++i) {
    irArray[0][1] += analogRead(LEFTIR_PIN);
    irArray[1][1] += analogRead(RIGHTIR_PIN);
    delay(IR_WAIT);
  }
  irArray[0][1] /= COLOUR_NO;
  irArray[1][1] /= COLOUR_NO;
  Serial.println("done.\n");

  // Save range
  // Serial.println(irArray[0][0]);
  // Serial.println(irArray[0][1]);
  // Serial.println(irArray[1][0]);
  // Serial.println(irArray[1][1]);
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
  Serial.println("===== CALIBRATING COLOR SENSORS (TOP) =====");
  int whiteArray[3] = {0};

  // Max values
  Serial.print("Put WHITE sample. Calibrating MAX in ");
  for (int i = CALLIBRATE_SEC; i > 0; --i) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; ++i) {
    led.setColor( // one-hot encoding
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
  for (int i = CALLIBRATE_SEC; i > 0; --i) {
    Serial.print(i); Serial.print(".. "); delay(1000);
  }
  for (int i = 0; i < 3; ++i) {
    led.setColor( // one-hot encoding
      ((1<<i)   &1) * LED_MAX,
      ((1<<i>>1)&1) * LED_MAX,
      ((1<<i>>2)&1) * LED_MAX
    ); led.show();
    delay(RGB_WAIT);
    
    blackArray[i] = 0;
    for (int j = 0; j < COLOUR_NO; ++j) {
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
