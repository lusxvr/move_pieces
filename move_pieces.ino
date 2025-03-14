#include <AccelStepper.h>

// --- Pin Assignments ---
// Step/Direction for X motor:
const int STEP_PIN_X = 3;
const int DIR_PIN_X  = 2;

// Step/Direction for Y motor:
const int STEP_PIN_Y = 5;
const int DIR_PIN_Y  = 4;

// End stop pins (pullup)
const int X_ENDSTOP_PIN = 6;  // Adjust as needed
const int Y_ENDSTOP_PIN = 7;  // Adjust as needed

const int MAGNET_PIN = 9;

// Hall Effect Pin Assignments
const int A1_PIN = 32;
const int A2_PIN = 30;
const int A3_PIN = 28;
const int A4_PIN = 26;
const int A5_PIN = 24;
const int A6_PIN = 22;

const int B1_PIN = 44;
const int B2_PIN = 42;
const int B3_PIN = 40;
const int B4_PIN = 38;
const int B5_PIN = 36;
const int B6_PIN = 34;

const int C1_PIN = 12;
const int C2_PIN = 10;
const int C3_PIN = 52;
const int C4_PIN = 50;
const int C5_PIN = 48;
const int C6_PIN = 46;

const int D1_PIN = 33;
const int D2_PIN = 31;
const int D3_PIN = 29;
const int D4_PIN = 27;
const int D5_PIN = 25;
const int D6_PIN = 23;

const int E1_PIN = 45;
const int E2_PIN = 43;
const int E3_PIN = 41;
const int E4_PIN = 39;
const int E5_PIN = 37;
const int E6_PIN = 35;

const int F1_PIN = 11;
const int F2_PIN = 13;
const int F3_PIN = 53;
const int F4_PIN = 51;
const int F5_PIN = 49;
const int F6_PIN = 47;

const int HALL_SENSOR_PINS[36] = {
  A1_PIN, A2_PIN, A3_PIN, A4_PIN, A5_PIN, A6_PIN,
  B1_PIN, B2_PIN, B3_PIN, B4_PIN, B5_PIN, B6_PIN,
  C1_PIN, C2_PIN, C3_PIN, C4_PIN, C5_PIN, C6_PIN,
  D1_PIN, D2_PIN, D3_PIN, D4_PIN, D5_PIN, D6_PIN,
  E1_PIN, E2_PIN, E3_PIN, E4_PIN, E5_PIN, E6_PIN,
  F1_PIN, F2_PIN, F3_PIN, F4_PIN, F5_PIN, F6_PIN
};

// Create two AccelStepper objects for X and Y
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// --- Movement parameters ---
const float STEPS_PER_MM_X = 100.0f / 15.0f;
const float STEPS_PER_MM_Y = 100.0f;

// Speed and Acceleration (in mm/s and mm/s^2, then converted to steps)
const float MAX_SPEED = 25;  // mm per second
const int   MAX_ACC   = 20;   // mm/s^2

// Homing
const int   MAX_HOMING_DIST   = 20000; // in mm
const float HOMING_RELEASE_MM = 4;     // how many mm to back off once the switch is triggered
const int HOMING_DIR_X = 1;
const int HOMING_DIR_Y = 1;

// Forward declarations
void homeAxis(AccelStepper &stepper, int endstopPin, float homeDirection);
void movePiece(float startXmm, float startYmm, float endXmm, float endYmm);
void moveXY(float xMM, float yMM);
String readBoardState();

void setup() {
  Serial.begin(115200);
  Serial.println("Chess Controller Starting");

  // Configure end stop pins as pullups
  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);

  // Configure magnet pin as output and ensure it starts OFF
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);  // Assuming LOW is OFF for your magnet

  // Initialize all hall effect sensor pins
  for (int i = 0; i < 36; i++) {
    pinMode(HALL_SENSOR_PINS[i], INPUT_PULLUP);
  }

  // Basic stepper configuration
  stepperX.setMaxSpeed(MAX_SPEED * STEPS_PER_MM_X);
  stepperX.setAcceleration(MAX_ACC * STEPS_PER_MM_X);

  stepperY.setMaxSpeed(MAX_SPEED * STEPS_PER_MM_Y);
  stepperY.setAcceleration(MAX_ACC * STEPS_PER_MM_Y);

  // --- HOME EACH AXIS ONCE ---
  homeAxis(stepperX, X_ENDSTOP_PIN, HOMING_DIR_X * STEPS_PER_MM_X);
  homeAxis(stepperY, Y_ENDSTOP_PIN, HOMING_DIR_Y * STEPS_PER_MM_Y);

  Serial.println("Homing complete. Type commands like:");
  Serial.println("  MOVE 5 5 10 10 - to move from (5mm,5mm) to (10mm,10mm).");
  Serial.println("  READ_BOARD - Get current board state from sensors");

}

void loop() {
  if (Serial.available() > 0) {
    String cmdString = Serial.readStringUntil('\n');
    cmdString.trim();
    if (cmdString.startsWith("MOVE")) {
      parseMoveCommand(cmdString);
    }
    else if (cmdString == "READ_BOARD") {
      // Read the current state of all sensors
      String boardState = readBoardState();
      
      // Send the board state back to the Python server
      Serial.println(boardState);
    }  
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmdString);
    }
  }
}

void parseMoveCommand(const String &cmdString) {
  // Example input:  "MOVE 5 5 10 10"
  // Remove the "MOVE" part
  // (Assumes "MOVE " is at the front; check length or do safer parsing in production code)
  String args = cmdString.substring(4);  // skip "MOVE"
  args.trim();                           // remove leading spaces

  // We expect four space-separated values, e.g. "5 5 10 10"
  // We'll split on spaces. The simplest approach is:
  float sx = 0, sy = 0, ex = 0, ey = 0;
  int spaceIndex;

  // 1) Parse sx
  spaceIndex = args.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("ERROR: Not enough arguments (sx).");
    return;
  }
  String sxStr = args.substring(0, spaceIndex);
  sx = sxStr.toFloat(); // convert to float

  // remove "sx" from args
  args.remove(0, spaceIndex + 1);

  // 2) Parse sy
  spaceIndex = args.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("ERROR: Not enough arguments (sy).");
    return;
  }
  String syStr = args.substring(0, spaceIndex);
  sy = syStr.toFloat();

  // remove "sy" from args
  args.remove(0, spaceIndex + 1);

  // 3) Parse ex
  spaceIndex = args.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("ERROR: Not enough arguments (ex).");
    return;
  }
  String exStr = args.substring(0, spaceIndex);
  ex = exStr.toFloat();

  // remove "ex" from args
  args.remove(0, spaceIndex + 1);

  // 4) Parse ey
  // The remainder of args is ey
  ey = args.toFloat();

  // Now we have sx, sy, ex, ey
  //Serial.println("Parsed MOVE command with floats:");
  //Serial.print("  Start = ("); Serial.print(sx); Serial.print(", "); Serial.print(sy);
  //Serial.print(") End = (");   Serial.print(ex); Serial.print(", "); Serial.print(ey);
  //Serial.println(")");

  // Call your function that does the move
  movePiece(sx, sy, ex, ey);
}

/**
 * movePiece - High-level function to move from (startXmm, startYmm)
 *             to (endXmm, endYmm). Turns magnet on/off (dummy),
 *             then re-homes.
 */
void movePiece(float startXmm, float startYmm, float endXmm, float endYmm) {
  //Serial.println("Moving to start (pick-up) position...");
  moveXY(startXmm, startYmm);
  delay(500);

  // Turn magnet ON
  //Serial.println("Magnet ON");
  digitalWrite(MAGNET_PIN, HIGH);  // Assuming HIGH is ON for your magnet
  delay(500);

  //Serial.println("Moving to end (drop-off) position...");
  moveXY(endXmm, endYmm);
  delay(500);

  // Turn magnet OFF
  //Serial.println("Magnet OFF");
  digitalWrite(MAGNET_PIN, LOW);  // Assuming LOW is OFF for your magnet
  delay(1500);

  // Re-home after the move
  homeAxis(stepperX, X_ENDSTOP_PIN, HOMING_DIR_X * STEPS_PER_MM_X);
  homeAxis(stepperY, Y_ENDSTOP_PIN, HOMING_DIR_Y * STEPS_PER_MM_Y);
  //Serial.println("Re-homing complete.");
  //Serial.println("OK"); // Here to acknowledge completed move to server
  Serial.println("MOVE_COMPLETE");
}

/**
 * moveXY - A blocking move function that commands
 *          both steppers to go to (xMM, yMM) in absolute mm coords.
 *          Then waits until both arrive.
 */
void moveXY(float xMM, float yMM) {
  // Convert mm to steps
  long xSteps = (long)(xMM * STEPS_PER_MM_X);
  long ySteps = (long)(yMM * STEPS_PER_MM_Y);

  /*
  Serial.print("moveXY: moving to X=");
  Serial.print(xMM); 
  Serial.print("mm ("); 
  Serial.print(xSteps); 
  Serial.print(" steps), Y=");
  Serial.print(yMM); 
  Serial.print("mm (");
  Serial.print(ySteps);
  Serial.println(" steps)");
  */

  // Command both steppers to move
  stepperX.moveTo(-xSteps);
  stepperY.moveTo(-ySteps);

  // Wait until both are finished
  while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0)) {
    stepperX.run();
    stepperY.run();
  }
  //Serial.println("...moveXY done.");
}

/**
 * homeAxis - Moves the stepper in a chosen direction until the end stop
 *            is triggered (LOW on an INPUT_PULLUP).
 *
 *  stepper       - reference to your AccelStepper object
 *  endstopPin    - Arduino pin with the end stop switch
 *  homeDirection - negative or positive steps per mm, indicating 
 *                  which way is "toward" the endstop.
 */
void homeAxis(AccelStepper &stepper, int endstopPin, float homeDirection) {
  //Serial.print("Homing axis on pin ");
  //Serial.println(endstopPin);

  // Start from a "fake" 0
  stepper.setCurrentPosition(0);

  // Move a big distance in the negative direction (or positive, as needed)
  long homingTargetSteps = (long)(homeDirection * MAX_HOMING_DIST);
  stepper.moveTo(homingTargetSteps);

  // Move until the switch goes LOW
  while (digitalRead(endstopPin) == HIGH) {
    stepper.run();
  }

  // Switch triggered, stop now
  stepper.stop();
  delay(500);

  // Mark this immediate position as 0
  stepper.setCurrentPosition(0);

  // Move away from switch a little so it's no longer pressed
  long releaseSteps = (long)(homeDirection * -1.0f *  HOMING_RELEASE_MM);

  stepper.moveTo(releaseSteps);

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.stop();

  // Finally, define this new position as 0 again
  stepper.setCurrentPosition(0);

  /*
  Serial.print("Axis on pin ");
  Serial.print(endstopPin);
  Serial.println(" homed. CurrentPosition set to 0.");
  */
}


// Function to read all hall effect sensors and return a 36-digit string
String readBoardState() {
  String boardState = "";
  
  for (int i = 0; i < 36; i++) {
    // Configure each pin as input with pullup resistor
    pinMode(HALL_SENSOR_PINS[i], INPUT_PULLUP);
    
    // Read the sensor (LOW typically means a piece is present due to magnet)
    // Note: You may need to invert this logic depending on your sensor setup
    int sensorValue = digitalRead(HALL_SENSOR_PINS[i]);
    
    // Convert to 1 (piece present) or 0 (empty)
    // Invert the reading since LOW typically means a piece is present
    boardState += (sensorValue == LOW) ? "1" : "0";
  }
  
  return boardState;
}
