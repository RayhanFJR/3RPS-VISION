//==================================================================
// ARDUINO MOTOR CONTROL SYSTEM - WITH ADAPTIVE LOAD CONTROL
// Mode: Manual, Auto Trajectory, Safety System, Retreat Communication
// Individual PID gains per motor (Outer + Inner Loop)
// Adaptive torque/speed based on load cell feedback
// Bidirectional communication with Minipc for trajectory retreat
//==================================================================

//==================================================================
// CONSTANTS & CONFIGURATION
//==================================================================

// Motor Control Speed
const int MANUAL_SPEED = 125;     // Manual mode speed (0-255)
const int RETREAT_SPEED = 150;    // Emergency retreat speed (0-255) - NOT USED in trajectory retreat

// Load Cell Configuration
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;
int threshold1 = 20;              // First threshold (modifiable via HMI)
int threshold2 = 40;              // Second threshold (modifiable via HMI)
long loadCellOffset = 0;
float latestValidLoad = 0.0;

// Adaptive Control Parameters
const float LOAD_SCALE_MIN = 0.15;        // Minimum speed multiplier (15%)
const float LOAD_SCALE_MAX = 1.0;         // Maximum speed multiplier (100%)
const float KP_DAMPING_FACTOR = 0.6;      // How much to reduce Kp (0.0-1.0)
const bool ENABLE_ADAPTIVE_MANUAL = true; // Enable for manual mode too
float smoothedLoad = 0.0;                 // Smoothed load reading
const float LOAD_ALPHA = 0.3;             // Smoothing factor (0.0-1.0)

// Retreat Control Parameters
const float RETREAT_VELOCITY_SCALE = 1.5; // Retreat 1.5x faster than forward
bool retreatRequestSent = false;          // Flag to prevent multiple RETREAT commands

// Motor Control Parameters
const float GR = 0.2786;          // Gear ratio
const float kt = 0.0663;          // Motor constant

// Current Sensor Parameters
const int adcMax = 1023;
const int nSamples = 3;
const float CalFac = 3.40;

// Timing Intervals (milliseconds)
const int loadCellInterval = 100;
const int encoderInterval = 1;
const int veloInterval = 10000;
const int CTCcalculationInterval = 100;
const int PDcalculationInterval = 100;

//==================================================================
// PIN DEFINITIONS
//==================================================================

// Motor PWM Pins
const int RPWM1 = 3, LPWM1 = 5;   // Motor 1
const int RPWM2 = 6, LPWM2 = 9;   // Motor 2
const int RPWM3 = 10, LPWM3 = 11; // Motor 3

// Encoder Pins
const int ENC1 = 4;
const int ENC2 = 2;
const int ENC3 = 8;

// Current Sensor Pins
const int CurrSen1 = A0;
const int CurrSen2 = A1;
const int CurrSen3 = A2;

//==================================================================
// SYSTEM STATE VARIABLES
//==================================================================

// Operating Mode
int operatingMode = 0;            // 0=Manual, 1=Auto Forward, 2=Auto Retreat
int manualCommand = 0;            // 0=Stop, 1=Forward, 2=Backward
int manipulatorState = 0;         // 0=Run, 1=Pause, 2=Retreat
bool retreatHasBeenTriggered = false;

// Communication Buffer
String receivedData = "";

//==================================================================
// MOTOR 1 VARIABLES
//==================================================================

// PID Gains
float kp1 = 110.0, kd1 = 0.1;     // Outer loop (position)
float kpc1 = 30.0, kdc1 = 0.1;    // Inner loop (current)

// Reference Values
float refPos1 = 0.0;
float refVelo1 = 0.0;
float refFc1 = 0.0;
float refCurrent1 = 0.0;

// Actual Values
float ActPos1 = 0.0;
float ActVelo1 = 0.0;
float ActCurrent1 = 0.0;
int position1 = 0;
int prevState1 = 0;

// Control Values
float controlValue1 = 0.0;
float ErrPos1 = 0.0;
float error1 = 0.0;
float prevError1 = 0.0;
float prevPos1 = 0.0;

//==================================================================
// MOTOR 2 VARIABLES
//==================================================================

// PID Gains
float kp2 = 142.0, kd2 = 0.6;
float kpc2 = 33.0, kdc2 = 0.1;

// Reference Values
float refPos2 = 0.0;
float refVelo2 = 0.0;
float refFc2 = 0.0;
float refCurrent2 = 0.0;

// Actual Values
float ActPos2 = 0.0;
float ActVelo2 = 0.0;
float ActCurrent2 = 0.0;
int position2 = 0;
int prevState2 = 0;

// Control Values
float controlValue2 = 0.0;
float ErrPos2 = 0.0;
float error2 = 0.0;
float prevError2 = 0.0;
float prevPos2 = 0.0;

//==================================================================
// MOTOR 3 VARIABLES
//==================================================================

// PID Gains
float kp3 = 150.0, kd3 = 0.3;
float kpc3 = 38.0, kdc3 = 0.1;

// Reference Values
float refPos3 = 0.0;
float refVelo3 = 0.0;
float refFc3 = 0.0;
float refCurrent3 = 0.0;

// Actual Values
float ActPos3 = 0.0;
float ActVelo3 = 0.0;
float ActCurrent3 = 0.0;
int position3 = 0;
int prevState3 = 0;

// Control Values
float controlValue3 = 0.0;
float ErrPos3 = 0.0;
float error3 = 0.0;
float prevError3 = 0.0;
float prevPos3 = 0.0;

//==================================================================
// TIMING VARIABLES
//==================================================================

long lastLoadTime = 0;
long lastEncTime = 0;
long lastVeloTime = 0;
long lastCTCCalcTime = 0;
long lastPDCalcTime = 0;
long lastPrnTime = 0;

//==================================================================
// ADAPTIVE CONTROL FUNCTIONS
//==================================================================

float getLoadScaling() {
  // Exponential smoothing untuk load reading
  smoothedLoad = (LOAD_ALPHA * latestValidLoad) + ((1.0 - LOAD_ALPHA) * smoothedLoad);
  
  if (smoothedLoad < threshold1) {
    // Normal operation - full power
    return LOAD_SCALE_MAX;
  } 
  else if (smoothedLoad >= threshold2) {
    // Maximum load - minimum power
    return LOAD_SCALE_MIN;
  } 
  else {
    // Linear interpolation between thresholds
    float loadRatio = (smoothedLoad - threshold1) / (float)(threshold2 - threshold1);
    return LOAD_SCALE_MAX - (loadRatio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN));
  }
}

float getAdaptiveKp(float baseKp) {
  if (smoothedLoad < threshold1) {
    return baseKp; // Normal Kp
  } 
  else if (smoothedLoad >= threshold2) {
    return baseKp * (1.0 - KP_DAMPING_FACTOR); // Reduced Kp
  } 
  else {
    float loadRatio = (smoothedLoad - threshold1) / (float)(threshold2 - threshold1);
    float damping = 1.0 - (loadRatio * KP_DAMPING_FACTOR);
    return baseKp * damping;
  }
}

//==================================================================
// CONTROL FUNCTIONS
//==================================================================

float Error(float ref, float act) {
  return ref - act;
}

float CTC(float PosError, float kp, float VeloError, float kd, float ForceID, float GR, float kt) {
  return ((PosError * kp) + (VeloError * kd) + ForceID) * GR * kt;
}

float PD(float error, float kpc, float errordev, float kdc) {
  return (error * kpc) + (errordev * kdc);
}

//==================================================================
// SENSOR READING FUNCTIONS
//==================================================================

long readHX711() {
  long result = 0;
  while (digitalRead(LOADCELL_DOUT_PIN));
  
  for (int i = 0; i < 24; i++) {
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    result = result << 1;
    if (digitalRead(LOADCELL_DOUT_PIN)) {
      result++;
    }
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);
  }
  
  digitalWrite(LOADCELL_SCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(LOADCELL_SCK_PIN, LOW);
  delayMicroseconds(1);
  
  if (result & 0x800000) {
    result |= ~0xFFFFFF;
  }
  
  return result;
}

float avg1() {
  float val1 = 0;
  for (int i = 0; i < nSamples; i++) {
    val1 += analogRead(CurrSen1);
    delay(1);
  }
  return val1 / adcMax / nSamples;
}

float avg2() {
  float val2 = 0;
  for (int i = 0; i < nSamples; i++) {
    val2 += analogRead(CurrSen2);
    delay(1);
  }
  return val2 / adcMax / nSamples;
}

float avg3() {
  float val3 = 0;
  for (int i = 0; i < nSamples; i++) {
    val3 += analogRead(CurrSen3);
    delay(1);
  }
  return val3 / adcMax / nSamples;
}

//==================================================================
// COMMAND PARSING FUNCTIONS
//==================================================================

void parseTrajectoryCommand(String data, bool isRetreat) {
  // Remove command prefix (S or R)
  data.replace("S", "");
  data.replace("R", "");
  
  int commaIndex1 = data.indexOf(',');
  String refPos1Str = data.substring(0, commaIndex1);
  data = data.substring(commaIndex1 + 1);
  
  int commaIndex2 = data.indexOf(',');
  String refPos2Str = data.substring(0, commaIndex2);
  data = data.substring(commaIndex2 + 1);
  
  int commaIndex3 = data.indexOf(',');
  String refPos3Str = data.substring(0, commaIndex3);
  data = data.substring(commaIndex3 + 1);
  
  int commaIndex4 = data.indexOf(',');
  String refVelo1Str = data.substring(0, commaIndex4);
  data = data.substring(commaIndex4 + 1);
  
  int commaIndex5 = data.indexOf(',');
  String refVelo2Str = data.substring(0, commaIndex5);
  data = data.substring(commaIndex5 + 1);
  
  int commaIndex6 = data.indexOf(',');
  String refVelo3Str = data.substring(0, commaIndex6);
  data = data.substring(commaIndex6 + 1);
  
  int commaIndex7 = data.indexOf(',');
  String refFc1Str = data.substring(0, commaIndex7);
  data = data.substring(commaIndex7 + 1);
  
  int commaIndex8 = data.indexOf(',');
  String refFc2Str = data.substring(0, commaIndex8);
  String refFc3Str = data.substring(commaIndex8 + 1);
  
  refPos1 = refPos1Str.toFloat();
  refPos2 = refPos2Str.toFloat();
  refPos3 = refPos3Str.toFloat();
  refVelo1 = refVelo1Str.toFloat();
  refVelo2 = refVelo2Str.toFloat();
  refVelo3 = refVelo3Str.toFloat();
  refFc1 = refFc1Str.toFloat();
  refFc2 = refFc2Str.toFloat();
  refFc3 = refFc3Str.toFloat();
  
  // If retreat mode, scale velocity for faster retreat
  if (isRetreat) {
    refVelo1 *= RETREAT_VELOCITY_SCALE;
    refVelo2 *= RETREAT_VELOCITY_SCALE;
    refVelo3 *= RETREAT_VELOCITY_SCALE;
  }
}

void parseOuterLoopGains(String data) {
  data.replace("K", "");
  
  int commaIndex1 = data.indexOf(',');
  String motorNumStr = data.substring(0, commaIndex1);
  data = data.substring(commaIndex1 + 1);
  
  int commaIndex2 = data.indexOf(',');
  String kpStr = data.substring(0, commaIndex2);
  String kdStr = data.substring(commaIndex2 + 1);
  
  int motorNum = motorNumStr.toInt();
  float newKp = kpStr.toFloat();
  float newKd = kdStr.toFloat();
  
  if (motorNum == 1) {
    kp1 = newKp;
    kd1 = newKd;
    Serial.print("Motor 1 Outer Loop gains updated: Kp=");
    Serial.print(kp1);
    Serial.print(", Kd=");
    Serial.println(kd1);
  } else if (motorNum == 2) {
    kp2 = newKp;
    kd2 = newKd;
    Serial.print("Motor 2 Outer Loop gains updated: Kp=");
    Serial.print(kp2);
    Serial.print(", Kd=");
    Serial.println(kd2);
  } else if (motorNum == 3) {
    kp3 = newKp;
    kd3 = newKd;
    Serial.print("Motor 3 Outer Loop gains updated: Kp=");
    Serial.print(kp3);
    Serial.print(", Kd=");
    Serial.println(kd3);
  } else {
    Serial.println("Invalid motor number for gain update.");
  }
  Serial.println("");
}

void parseInnerLoopGains(String data) {
  data.replace("P", "");
  
  int commaIndex1 = data.indexOf(',');
  String motorNumStr = data.substring(0, commaIndex1);
  data = data.substring(commaIndex1 + 1);
  
  int commaIndex2 = data.indexOf(',');
  String kpcStr = data.substring(0, commaIndex2);
  String kdcStr = data.substring(commaIndex2 + 1);
  
  int motorNum = motorNumStr.toInt();
  float newKpc = kpcStr.toFloat();
  float newKdc = kdcStr.toFloat();
  
  if (motorNum == 1) {
    kpc1 = newKpc;
    kdc1 = newKdc;
    Serial.print("Motor 1 Inner Loop gains updated: Kpc=");
    Serial.print(kpc1);
    Serial.print(", Kdc=");
    Serial.println(kdc1);
  } else if (motorNum == 2) {
    kpc2 = newKpc;
    kdc2 = newKdc;
    Serial.print("Motor 2 Inner Loop gains updated: Kpc=");
    Serial.print(kpc2);
    Serial.print(", Kdc=");
    Serial.println(kdc2);
  } else if (motorNum == 3) {
    kpc3 = newKpc;
    kdc3 = newKdc;
    Serial.print("Motor 3 Inner Loop gains updated: Kpc=");
    Serial.print(kpc3);
    Serial.print(", Kdc=");
    Serial.println(kdc3);
  } else {
    Serial.println("Invalid motor number for gain update.");
  }
  Serial.println("");
}

void parseThresholds(String data) {
  data.replace("T", "");
  
  int commaIndex = data.indexOf(',');
  String t1Str = data.substring(0, commaIndex);
  String t2Str = data.substring(commaIndex + 1);
  
  threshold1 = t1Str.toInt();
  threshold2 = t2Str.toInt();
  
  Serial.print("Thresholds updated: T1=");
  Serial.print(threshold1);
  Serial.print(", T2=");
  Serial.println(threshold2);
  Serial.println("Adaptive scaling will adjust based on new thresholds");
  Serial.println("");
}

void resetSystem() {
  // Stop all motors
  operatingMode = 0;
  manualCommand = 0;
  manipulatorState = 0;
  retreatHasBeenTriggered = false;
  retreatRequestSent = false;
  
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
  
  // Tare load cell
  loadCellOffset = readHX711();
  
  // Reset smoothed load
  smoothedLoad = 0.0;
  
  // Reset all position and error variables
  position1 = 0; position2 = 0; position3 = 0;
  ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
  prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
  ErrPos1 = 0.0; ErrPos2 = 0.0; ErrPos3 = 0.0;
  error1 = 0.0; error2 = 0.0; error3 = 0.0;
  prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;
  ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;
  
  Serial.println("System Zeroed: Tare & Position Reset OK\n");
}

void emergencyStop() {
  operatingMode = 0;
  manualCommand = 0;
  retreatHasBeenTriggered = false;
  retreatRequestSent = false;
  
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
  
  Serial.println("EMERGENCY_STOP");
}

//==================================================================
// MOTOR CONTROL FUNCTIONS
//==================================================================

void updateEncoders() {
  int currentState1 = digitalRead(ENC1);
  if (currentState1 > prevState1) {
    if (controlValue1 > 0) position1++;
    else if (controlValue1 < 0) position1--;
  }
  ActPos1 = position1 * 0.245;
  prevState1 = currentState1;
  
  int currentState2 = digitalRead(ENC2);
  if (currentState2 > prevState2) {
    if (controlValue2 > 0) position2++;
    else if (controlValue2 < 0) position2--;
  }
  ActPos2 = position2 * 0.245;
  prevState2 = currentState2;
  
  int currentState3 = digitalRead(ENC3);
  if (currentState3 > prevState3) {
    if (controlValue3 > 0) position3++;
    else if (controlValue3 < 0) position3--;
  }
  ActPos3 = position3 * 0.245;
  prevState3 = currentState3;
}

void updateVelocities() {
  ActVelo1 = (ActPos1 - prevPos1) / 10;
  ActVelo2 = (ActPos2 - prevPos2) / 10;
  ActVelo3 = (ActPos3 - prevPos3) / 10;
  prevPos1 = ActPos1;
  prevPos2 = ActPos2;
  prevPos3 = ActPos3;
}

void calculateCTC() {
  ErrPos1 = Error(refPos1, ActPos1);
  ErrPos2 = Error(refPos2, ActPos2);
  ErrPos3 = Error(refPos3, ActPos3);
  
  float ErrVelo1 = Error(refVelo1, ActVelo1);
  float ErrVelo2 = Error(refVelo2, ActVelo2);
  float ErrVelo3 = Error(refVelo3, ActVelo3);
  
  // *** ADAPTIVE GAINS BASED ON LOAD ***
  float kp1_adaptive = getAdaptiveKp(kp1);
  float kp2_adaptive = getAdaptiveKp(kp2);
  float kp3_adaptive = getAdaptiveKp(kp3);
  
  refCurrent1 = CTC(ErrPos1, kp1_adaptive, ErrVelo1, kd1, refFc1, GR, kt);
  refCurrent2 = CTC(ErrPos2, kp2_adaptive, ErrVelo2, kd2, refFc2, GR, kt);
  refCurrent3 = CTC(ErrPos3, kp3_adaptive, ErrVelo3, kd3, refFc3, GR, kt);
}

void calculatePD() {
  error1 = Error(refCurrent1, ActCurrent1);
  error2 = Error(refCurrent2, ActCurrent2);
  error3 = Error(refCurrent3, ActCurrent3);
  
  float DerError1 = (error1 - prevError1) / 0.1;
  float DerError2 = (error2 - prevError2) / 0.1;
  float DerError3 = (error3 - prevError3) / 0.1;
  
  controlValue1 = PD(error1, kpc1, DerError1, kdc1);
  controlValue2 = PD(error2, kpc2, DerError2, kdc2);
  controlValue3 = PD(error3, kpc3, DerError3, kdc3);
  
  prevError1 = error1;
  prevError2 = error2;
  prevError3 = error3;
  
  // *** APPLY LOAD-BASED SPEED SCALING (only in forward mode) ***
  if (operatingMode == 1) {
    float loadScale = getLoadScaling();
    controlValue1 = constrain(controlValue1 * loadScale, -255, 255);
    controlValue2 = constrain(controlValue2 * loadScale, -255, 255);
    controlValue3 = constrain(controlValue3 * loadScale, -255, 255);
  } else {
    // Retreat mode: no adaptive scaling, full speed
    controlValue1 = constrain(controlValue1, -255, 255);
    controlValue2 = constrain(controlValue2, -255, 255);
    controlValue3 = constrain(controlValue3, -255, 255);
  }
}

void applyMotorControl() {
  // Motor 1
  if (ErrPos1 > 0) {
    analogWrite(RPWM1, abs(controlValue1));
    analogWrite(LPWM1, 0);
  } else if (ErrPos1 < 0) {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, abs(controlValue1));
  } else {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
  }
  
  // Motor 2
  if (ErrPos2 > 0) {
    analogWrite(RPWM2, abs(controlValue2));
    analogWrite(LPWM2, 0);
  } else if (ErrPos2 < 0) {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, abs(controlValue2));
  } else {
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
  }
  
  // Motor 3
  if (ErrPos3 > 0) {
    analogWrite(RPWM3, abs(controlValue3));
    analogWrite(LPWM3, 0);
  } else if (ErrPos3 < 0) {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, abs(controlValue3));
  } else {
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
  }
}

void stopAllMotors() {
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
}

void manualModeControl() {
  int effectiveSpeed = MANUAL_SPEED;
  
  // Apply adaptive scaling to manual mode if enabled
  if (ENABLE_ADAPTIVE_MANUAL) {
    float loadScale = getLoadScaling();
    effectiveSpeed = MANUAL_SPEED * loadScale;
  }
  
  if (manualCommand == 1) {
    // Forward
    analogWrite(RPWM1, effectiveSpeed); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, effectiveSpeed); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, effectiveSpeed); analogWrite(LPWM3, 0);
  } else if (manualCommand == 2) {
    // Backward
    analogWrite(RPWM1, 0); analogWrite(LPWM1, effectiveSpeed);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, effectiveSpeed);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, effectiveSpeed);
  } else {
    // Stop
    stopAllMotors();
  }
}

//==================================================================
// SETUP
//==================================================================

void setup() {
  Serial.begin(115200);
  
  // Motor PWM pins
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM3, OUTPUT);
  pinMode(LPWM3, OUTPUT);
  
  // Encoder pins
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(ENC3, INPUT);
  
  // Current sensor pins
  pinMode(CurrSen1, INPUT);
  pinMode(CurrSen2, INPUT);
  pinMode(CurrSen3, INPUT);
  
  // Load cell pins
  pinMode(LOADCELL_DOUT_PIN, INPUT);
  pinMode(LOADCELL_SCK_PIN, OUTPUT);
  
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("===========================================");
  Serial.println("Arduino Adaptive Motor Control System");
  Serial.println("Mode: Manual/Auto with Retreat Communication");
  Serial.println("===========================================");
  Serial.println("");
}

//==================================================================
// MAIN LOOP
//==================================================================

void loop() {
  long currentTime = millis();
  
  //================================================================
  // SERIAL COMMAND PROCESSING
  //================================================================
  
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;
    
    if (receivedChar == '\n') {
      receivedData.trim();
      
      // Forward trajectory command (S)
      if (receivedData.startsWith("S")) {
        operatingMode = 1;  // Auto forward mode
        retreatHasBeenTriggered = false;
        retreatRequestSent = false;
        manipulatorState = 0;
        parseTrajectoryCommand(receivedData, false);
      }
      // Retreat trajectory command (R with positions)
      else if (receivedData.startsWith("R") && receivedData.indexOf(',') > 0) {
        operatingMode = 2;  // Auto retreat mode
        manipulatorState = 0;  // Running (not pause/retreat state)
        parseTrajectoryCommand(receivedData, true);
      }
      // Retreat finished notification from Minipc
      else if (receivedData == "RETREAT_COMPLETE") {
        operatingMode = 0;
        manualCommand = 0;
        retreatHasBeenTriggered = false;
        retreatRequestSent = false;
        stopAllMotors();
        Serial.println("ACK_RETREAT_COMPLETE");
      }
      // Tare and reset command (X)
      else if (receivedData.startsWith("X")) {
        resetSystem();
      }
      // Emergency stop (E)
      else if (receivedData.startsWith("E")) {
        emergencyStop();
      }
      // Manual forward (1)
      else if (receivedData == "1") {
        operatingMode = 0;
        manualCommand = 1;
        retreatHasBeenTriggered = false;
        retreatRequestSent = false;
      }
      // Manual backward (2)
      else if (receivedData == "2") {
        operatingMode = 0;
        manualCommand = 2;
        retreatHasBeenTriggered = false;
        retreatRequestSent = false;
      }
      // Manual stop (0)
      else if (receivedData == "0") {
        operatingMode = 0;
        manualCommand = 0;
      }
      // Set outer loop gains (K)
      else if (receivedData.startsWith("K")) {
        operatingMode = 0;
        manualCommand = 0;
        parseOuterLoopGains(receivedData);
      }
      // Set inner loop gains (P)
      else if (receivedData.startsWith("P")) {
        operatingMode = 0;
        manualCommand = 0;
        parseInnerLoopGains(receivedData);
      }
      // Set thresholds (T)
      else if (receivedData.startsWith("T")) {
        operatingMode = 0;
        manualCommand = 0;
        parseThresholds(receivedData);
      }
      
      receivedData = "";
    }
  }
  
  //================================================================
  // AUTO MODE EXECUTION (FORWARD OR RETREAT)
  //================================================================
  
  if (operatingMode == 1 || operatingMode == 2) {
    
    // Load cell monitoring (only in forward mode)
    if (operatingMode == 1 && currentTime - lastLoadTime >= loadCellInterval) {
      if (retreatHasBeenTriggered) {
        manipulatorState = 1;  // Pause
      } else {
        long effectiveValue = readHX711() - loadCellOffset;
        latestValidLoad = effectiveValue / 10000.0;
        int roundValue = round(latestValidLoad);
        
        if (roundValue >= threshold2) {
          // Trigger retreat - notify Minipc
          manipulatorState = 1;  // Pause current movement
          retreatHasBeenTriggered = true;
          
          if (!retreatRequestSent) {
            Serial.println("RETREAT");
            retreatRequestSent = true;
          }
        } else if (roundValue >= threshold1) {
          manipulatorState = 1;  // Pause
        } else {
          manipulatorState = 0;  // Run
        }
      }
      lastLoadTime = currentTime;
    }
    
    // Encoder reading
    if (currentTime - lastEncTime >= encoderInterval && manipulatorState != 1) {
      updateEncoders();
      lastEncTime = currentTime;
    }
    
    // Control calculations (only in running state)
    if (manipulatorState == 0) {
      
      // Velocity calculation
      if (currentTime - lastVeloTime >= veloInterval) {
        updateVelocities();
        lastVeloTime = currentTime;
      }
      
      // CTC (outer loop) calculation with adaptive gains
      if (currentTime - lastCTCCalcTime >= CTCcalculationInterval) {
        calculateCTC();
        lastCTCCalcTime = currentTime;
      }
      
      // PD (inner loop) calculation with adaptive scaling
      if (currentTime - lastPDCalcTime >= PDcalculationInterval) {
        calculatePD();
        lastPDCalcTime = currentTime;
      }
    }
    
    // Motor control execution
    if (manipulatorState == 0) {
      applyMotorControl();
    } else {
      stopAllMotors();
    }
    
    // Status reporting
    if (currentTime - lastPrnTime >= loadCellInterval) {
      String statusStr;
      String modeStr;
      
      if (operatingMode == 1) {
        modeStr = "forward";
      } else {
        modeStr = "retreat";
      }
      
      if (manipulatorState == 0) {
        statusStr = "running";
      } else if (manipulatorState == 1) {
        statusStr = "paused";
      } else if (manipulatorState == 2) {
        statusStr = "retreating";
      }
      
      float currentScale = getLoadScaling();
      Serial.print("status:");
      Serial.print(statusStr);
      Serial.print(",mode:");
      Serial.print(modeStr);
      Serial.print(",load:");
      Serial.print(latestValidLoad, 2);
      Serial.print(",scale:");
      Serial.print(currentScale, 2);
      Serial.print(",pos:");
      Serial.print(ActPos1, 2);
      Serial.print(",");
      Serial.print(ActPos2, 2);
      Serial.print(",");
      Serial.println(ActPos3, 2);
      
      lastPrnTime = currentTime;
    }
    
  }
  
  //================================================================
  // MANUAL MODE EXECUTION (WITH ADAPTIVE CONTROL)
  //================================================================
  
  else {
    // Read load cell even in manual mode for adaptive control
    if (currentTime - lastLoadTime >= loadCellInterval && ENABLE_ADAPTIVE_MANUAL) {
      long effectiveValue = readHX711() - loadCellOffset;
      latestValidLoad = effectiveValue / 10000.0;
      lastLoadTime = currentTime;
    }
    
    manualModeControl();
  }
}