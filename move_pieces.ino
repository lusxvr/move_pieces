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

const int MAGNET_PIN = 10;

// Create two AccelStepper objects for X and Y
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// --- Movement parameters ---
const float STEPS_PER_MM_X = 100.0f / 15.0f;
const float STEPS_PER_MM_Y = 100.0f;

// Speed and Acceleration (in mm/s and mm/s^2, then converted to steps)
const float MAX_SPEED = 10;  // mm per second
const int   MAX_ACC   = 5;   // mm/s^2

// Homing
const int   MAX_HOMING_DIST   = 20000; // in mm
const float HOMING_RELEASE_MM = 4;     // how many mm to back off once the switch is triggered
const int HOMING_DIR_X = 1;
const int HOMING_DIR_Y = 1;

// Forward declarations
void homeAxis(AccelStepper &stepper, int endstopPin, float homeDirection);
void movePiece(float startXmm, float startYmm, float endXmm, float endYmm);
void moveXY(float xMM, float yMM);

void setup() {
  Serial.begin(115200);
  Serial.println("AccelStepper Homing + Move Demo Beginning...");

  // Configure end stop pins as pullups
  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);

  // Configure magnet pin as output and ensure it starts OFF
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);  // Assuming LOW is OFF for your magnet

  // Basic stepper configuration
  stepperX.setMaxSpeed(MAX_SPEED * STEPS_PER_MM_X);
  stepperX.setAcceleration(MAX_ACC * STEPS_PER_MM_X);

  stepperY.setMaxSpeed(MAX_SPEED * STEPS_PER_MM_Y);
  stepperY.setAcceleration(MAX_ACC * STEPS_PER_MM_Y);

  // --- HOME EACH AXIS ONCE ---
  homeAxis(stepperX, X_ENDSTOP_PIN, HOMING_DIR_X * STEPS_PER_MM_X);
  homeAxis(stepperY, Y_ENDSTOP_PIN, HOMING_DIR_Y * STEPS_PER_MM_Y);

  Serial.println("Homing complete. Type commands like:");
  Serial.println("  MOVE 5 5 10 10");
  Serial.println("to move from (5mm,5mm) to (10mm,10mm).");
}

void loop() {
  if (Serial.available() > 0) {
    String cmdString = Serial.readStringUntil('\n');
    cmdString.trim();
    if (cmdString.startsWith("MOVE")) {
      parseMoveCommand(cmdString);
    } else {
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
  Serial.println("Parsed MOVE command with floats:");
  Serial.print("  Start = ("); Serial.print(sx); Serial.print(", "); Serial.print(sy);
  Serial.print(") End = (");   Serial.print(ex); Serial.print(", "); Serial.print(ey);
  Serial.println(")");

  // Call your function that does the move
  movePiece(sx, sy, ex, ey);
}

/**
 * movePiece - High-level function to move from (startXmm, startYmm)
 *             to (endXmm, endYmm). Turns magnet on/off (dummy),
 *             then re-homes.
 */
void movePiece(float startXmm, float startYmm, float endXmm, float endYmm) {
  Serial.println("Moving to start (pick-up) position...");
  moveXY(startXmm, startYmm);
  delay(500);

  // Turn magnet ON
  Serial.println("Magnet ON");
  digitalWrite(MAGNET_PIN, HIGH);  // Assuming HIGH is ON for your magnet
  delay(500);

  Serial.println("Moving to end (drop-off) position...");
  moveXY(endXmm, endYmm);
  delay(500);

  // Turn magnet OFF
  Serial.println("Magnet OFF");
  digitalWrite(MAGNET_PIN, LOW);  // Assuming LOW is OFF for your magnet
  delay(1500);

  // Re-home after the move
  Serial.println("Re-homing after move...");
  homeAxis(stepperX, X_ENDSTOP_PIN, HOMING_DIR_X * STEPS_PER_MM_X);
  homeAxis(stepperY, Y_ENDSTOP_PIN, HOMING_DIR_Y * STEPS_PER_MM_Y);
  Serial.println("Re-homing complete.");
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

  Serial.print("moveXY: moving to X=");
  Serial.print(xMM); 
  Serial.print("mm ("); 
  Serial.print(xSteps); 
  Serial.print(" steps), Y=");
  Serial.print(yMM); 
  Serial.print("mm (");
  Serial.print(ySteps);
  Serial.println(" steps)");

  // Command both steppers to move
  stepperX.moveTo(-xSteps);
  stepperY.moveTo(-ySteps);

  // Wait until both are finished
  while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0)) {
    stepperX.run();
    stepperY.run();
  }
  Serial.println("...moveXY done.");
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
  Serial.print("Homing axis on pin ");
  Serial.println(endstopPin);

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

  Serial.print("Axis on pin ");
  Serial.print(endstopPin);
  Serial.println(" homed. CurrentPosition set to 0.");
}
