//==================================================================
// KODE LENGKAP ARDUINO (MANUAL, TARE, AUTO, SAFETY) - FINAL
// VERSI DENGAN KP, KD, KPC, & KDC INDIVIDUAL PER MOTOR
//==================================================================

// ----------- PENGATURAN UMUM -----------
const int MANUAL_SPEED = 125;  // Kecepatan untuk gerak manual (0-255)
const int RETREAT_SPEED = 150; // Kecepatan saat mundur darurat (0-255)

// ----------- MODE & STATE -----------
int operatingMode = 0;         // 0 = Manual Mode, 1 = Auto/Trajectory Mode
int manualCommand = 0;         // Perintah manual terakhir: 0=Stop, 1=Maju, 2=Mundur
int manipulatorState = 0;      // State internal saat mode Auto: 0=Run, 1=Pause, 2=Retreat
bool retreatHasBeenTriggered = false;

// ----------- PARAMETER LOAD CELL -----------
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;
int threshold1 = 20;
int threshold2 = 40;
long loadCellOffset = 0;

// ----------- VARIABEL KOMUNIKASI & KONTROL -----------
String receivedData = "";
float refPos1 = 0.0, refPos2 = 0.0, refPos3 = 0.0;
float refVelo1 = 0.0, refVelo2 = 0.0, refVelo3 = 0.0;
float refFc1 = 0.0, refFc2 = 0.0, refFc3 = 0.0;
float controlValue1 = 0.0, controlValue2 = 0.0, controlValue3 = 0.0;
float ErrPos1 = 0.0, ErrPos2 = 0.0, ErrPos3 = 0.0;
float ActPos1 = 0.0, ActPos2 = 0.0, ActPos3 = 0.0;
float ActVelo1 = 0.0, ActVelo2 = 0.0, ActVelo3 = 0.0;
float prevPos1 = 0.0, prevPos2 = 0.0, prevPos3 = 0.0;
float error1 = 0.0, error2 = 0.0, error3 = 0.0;
float prevError1 = 0.0, prevError2 = 0.0, prevError3 = 0.0;
float refCurrent1 = 0.0, refCurrent2 = 0.0, refCurrent3 = 0.0;
float ActCurrent1 = 0.0, ActCurrent2 = 0.0, ActCurrent3 = 0.0;
int position1 = 0, position2 = 0, position3 = 0;
int prevState1 = 0, prevState2 = 0, prevState3 = 0;

// <<< PERBAIKAN 1: Tambahkan variabel ini untuk menyimpan nilai beban terakhir >>>
float latestValidLoad = 0.0;

// Variabel kontrol umum
float GR = 0.2786, kt = 0.0663;

// --- Gain Outer Loop (CTC) - per Motor ---
float kp1 = 110.0, kd1 = 0.1; // Gain untuk Motor 1
float kp2 = 142.0, kd2 = 0.6;  // Gain untuk Motor 2
float kp3 = 150.0, kd3 = 0.3;  // Gain untuk Motor 3

// --- Gain Inner Loop (PD Current) - per Motor ---
float kpc1 = 30.0, kdc1 = 0.1; // Gain untuk Motor 1
float kpc2 = 33.0, kdc2 = 0.1; // Gain untuk Motor 2
float kpc3 = 38.0, kdc3 = 0.1; // Gain untuk Motor 3

//Parameter Sensor Arus
int adcMax = 1023;
int nSamples = 3;
float CalFac = 3.40;

// ----------- NOMOR PIN -----------
const int RPWM1 = 3, LPWM1 = 5;
const int RPWM2 = 6, LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;
const int ENC1 = 4, ENC2 = 2, ENC3 = 8;
const int CurrSen1 = A0;
const int CurrSen2 = A1;
const int CurrSen3 = A2;

// ----------- TIMER -----------
const int loadCellInterval = 100;
const int encoderInterval = 1;
const int veloInterval = 10000;
const int CTCcalculationInterval = 100;
const int PDcalculationInterval = 100;
long lastLoadTime = 0, lastEncTime = 0, lastVeloTime = 0, lastCTCCalcTime = 0, lastPDCalcTime = 0, lastPrnTime = 0;

// ----------- FUNGSI-FUNGSI -----------
float Error(float ref, float act) {
  return ref - act;
}

float CTC(float PosError, float kp, float VeloError, float kd, float ForceID, float GR, float kt) {
  return ((PosError * kp) + (VeloError * kd) + ForceID) * GR * kt;
}

float PD(float error, float kpc, float errordev, float kdc) {
  return (error * kpc) + (errordev * kdc);
}

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

void setup() {
  Serial.begin(115200);
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM3, OUTPUT);
  pinMode(LPWM3, OUTPUT);
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(ENC3, INPUT);
  pinMode(CurrSen1, INPUT);
  pinMode(CurrSen2, INPUT);
  pinMode(CurrSen3, INPUT);
  pinMode(LOADCELL_DOUT_PIN, INPUT);
  pinMode(LOADCELL_SCK_PIN, OUTPUT);
  while (!Serial) {
    ;
  }
}

void loop() {
  long currentTime = millis();

  // 1. INTERPRETER PERINTAH SERIAL
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;
    if (receivedChar == '\n') {
      receivedData.trim();

      if (receivedData.startsWith("S")) {
        operatingMode = 1;
        retreatHasBeenTriggered = false;
        manipulatorState = 0;
        receivedData.replace("S", "");
        int commaIndex1 = receivedData.indexOf(',');
        String refPos1Str = receivedData.substring(0, commaIndex1);
        receivedData = receivedData.substring(commaIndex1 + 1);
        int commaIndex2 = receivedData.indexOf(',');
        String refPos2Str = receivedData.substring(0, commaIndex2);
        receivedData = receivedData.substring(commaIndex2 + 1);
        int commaIndex3 = receivedData.indexOf(',');
        String refPos3Str = receivedData.substring(0, commaIndex3);
        receivedData = receivedData.substring(commaIndex3 + 1);
        int commaIndex4 = receivedData.indexOf(',');
        String refVelo1Str = receivedData.substring(0, commaIndex4);
        receivedData = receivedData.substring(commaIndex4 + 1);
        int commaIndex5 = receivedData.indexOf(',');
        String refVelo2Str = receivedData.substring(0, commaIndex5);
        receivedData = receivedData.substring(commaIndex5 + 1);
        int commaIndex6 = receivedData.indexOf(',');
        String refVelo3Str = receivedData.substring(0, commaIndex6);
        receivedData = receivedData.substring(commaIndex6 + 1);
        int commaIndex7 = receivedData.indexOf(',');
        String refFc1Str = receivedData.substring(0, commaIndex7);
        receivedData = receivedData.substring(commaIndex7 + 1);
        int commaIndex8 = receivedData.indexOf(',');
        String refFc2Str = receivedData.substring(0, commaIndex8);
        String refFc3Str = receivedData.substring(commaIndex8 + 1);

        refPos1 = refPos1Str.toFloat();
        refPos2 = refPos2Str.toFloat();
        refPos3 = refPos3Str.toFloat();
        refVelo1 = refVelo1Str.toFloat();
        refVelo2 = refVelo2Str.toFloat();
        refVelo3 = refVelo3Str.toFloat();
        refFc1 = refFc1Str.toFloat();
        refFc2 = refFc2Str.toFloat();
        refFc3 = refFc3Str.toFloat();

      } else if (receivedData.startsWith("X")) {
        // 1. Hentikan semua gerakan motor yang mungkin masih aktif
        operatingMode = 0;
        manualCommand = 0;
        manipulatorState = 0;
        
        analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
        analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
        analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);

        // 2. Kalibrasi Load Cell (Tare)
        loadCellOffset = readHX711();

        // 3. Reset SEMUA variabel posisi dan error ke NOL
        position1 = 0; position2 = 0; position3 = 0;
        ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
        prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
        ErrPos1 = 0.0; ErrPos2 = 0.0; ErrPos3 = 0.0;
        error1 = 0.0; error2 = 0.0; error3 = 0.0;
        prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;
        ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;

        // 4. Kirim konfirmasi yang lebih jelas
        Serial.println("System Zeroed: Tare & Position Reset OK\n");
        
      } else if (receivedData.startsWith("E")) { // Perintah Emergency
        operatingMode = 0; // Keluar dari mode otomatis
        manualCommand = 0; // Hentikan semua perintah manual
        // Langsung matikan semua motor
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, 0);
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, 0);
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, 0);
        // Arduino sekarang dalam keadaan berhenti total.

      } else if (receivedData.startsWith("R")) { // Perintah Reset
        operatingMode = 0; // Pastikan tetap di mode manual
        manualCommand = 2; // Atur perintah untuk mundur (seperti tombol "Backward")
        manipulatorState = 0;
        
        // Logika di bagian akhir loop() akan menangani pergerakan mundurnya.
        retreatHasBeenTriggered = false;
      } else if (receivedData == "1") {
        operatingMode = 0;
        manualCommand = 1;
      } else if (receivedData == "2") {
        operatingMode = 0;
        manualCommand = 2;
      } else if (receivedData == "0") {
        operatingMode = 0;
        manualCommand = 0;
      } else if (receivedData.startsWith("K")) { // Set Kp & Kd (Outer Loop)
        operatingMode = 0;
        manualCommand = 0;
        receivedData.replace("K", "");
        int commaIndex1 = receivedData.indexOf(',');
        String motorNumStr = receivedData.substring(0, commaIndex1);
        receivedData = receivedData.substring(commaIndex1 + 1);
        int commaIndex2 = receivedData.indexOf(',');
        String kpStr = receivedData.substring(0, commaIndex2);
        String kdStr = receivedData.substring(commaIndex2 + 1);
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
      } else if (receivedData.startsWith("P")) { // Set Kpc & Kdc (Inner Loop)
        operatingMode = 0;
        manualCommand = 0;
        receivedData.replace("P", "");
        int commaIndex1 = receivedData.indexOf(',');
        String motorNumStr = receivedData.substring(0, commaIndex1);
        receivedData = receivedData.substring(commaIndex1 + 1);
        int commaIndex2 = receivedData.indexOf(',');
        String kpcStr = receivedData.substring(0, commaIndex2);
        String kdcStr = receivedData.substring(commaIndex2 + 1);
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
      
        } else if (receivedData.startsWith("T")) { // Perintah Set Threshold
        operatingMode = 0; // Masuk mode manual untuk keamanan
        manualCommand = 0;
        receivedData.replace("T", "");

        int commaIndex = receivedData.indexOf(',');
        String t1Str = receivedData.substring(0, commaIndex);
        String t2Str = receivedData.substring(commaIndex + 1);

        threshold1 = t1Str.toInt();
        threshold2 = t2Str.toInt();

        Serial.print("Thresholds updated: T1=");
        Serial.print(threshold1);
        Serial.print(", T2=");
        Serial.println(threshold2);
        Serial.println("");
      }
      receivedData = "";
    }
  }

  // 2. EKSEKUSI BERDASARKAN MODE
  if (operatingMode == 1) {
    if (currentTime - lastLoadTime >= loadCellInterval) {
      if (retreatHasBeenTriggered) {
        manipulatorState = 2;
      } else {
        long effectiveValue = readHX711() - loadCellOffset;
        latestValidLoad = effectiveValue / 10000.0; // <-- BARIS INI DITAMBAHKAN
        int roundValue = round(latestValidLoad);
        if (roundValue >= threshold2) {
          manipulatorState = 2;
          retreatHasBeenTriggered = true;
        } else if (roundValue >= threshold1) {
          manipulatorState = 1;
        } else {
          manipulatorState = 0;
        }
      }
      lastLoadTime = currentTime;
    }

    if (currentTime - lastEncTime >= encoderInterval && manipulatorState != 1) {
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
      lastEncTime = currentTime;

      if ((ActPos1 == 0) && (ActPos2 == 0) && (ActPos3 == 0)){
        String statusStr;
        statusStr = "aman";
      }
    }

    if (manipulatorState == 0) {
      if (currentTime - lastVeloTime >= veloInterval) {
        ActVelo1 = (ActPos1 - prevPos1) / 10;
        ActVelo2 = (ActPos2 - prevPos2) / 10;
        ActVelo3 = (ActPos3 - prevPos3) / 10;
        prevPos1 = ActPos1;
        prevPos2 = ActPos2;
        prevPos3 = ActPos3;
        lastVeloTime = currentTime;
      }
      if (currentTime - lastCTCCalcTime >= CTCcalculationInterval) {
        ErrPos1 = Error(refPos1, ActPos1);
        ErrPos2 = Error(refPos2, ActPos2);
        ErrPos3 = Error(refPos3, ActPos3);
        float ErrVelo1 = Error(refVelo1, ActVelo1);
        float ErrVelo2 = Error(refVelo2, ActVelo2);
        float ErrVelo3 = Error(refVelo3, ActVelo3);
        refCurrent1 = CTC(ErrPos1, kp1, ErrVelo1, kd1, refFc1, GR, kt);
        refCurrent2 = CTC(ErrPos2, kp2, ErrVelo2, kd2, refFc2, GR, kt);
        refCurrent3 = CTC(ErrPos3, kp3, ErrVelo3, kd3, refFc3, GR, kt);
        lastCTCCalcTime = currentTime;
      }
      if (currentTime - lastPDCalcTime >= PDcalculationInterval) {
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
        controlValue1 = constrain(controlValue1, -255, 255);
        controlValue2 = constrain(controlValue2, -255, 255);
        controlValue3 = constrain(controlValue3, -255, 255);
        lastPDCalcTime = currentTime;
      }
    }

    if (manipulatorState == 0) {
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
    } else if (manipulatorState == 2) {
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, RETREAT_SPEED);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, RETREAT_SPEED);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, RETREAT_SPEED);
    } else {
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, 0);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, 0);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, 0);
    }

    if (currentTime - lastPrnTime >= loadCellInterval) {
      String statusStr;
      if (manipulatorState == 0) {
        statusStr = "running";
      } else if (manipulatorState == 1) {
        statusStr = "paused";
      } else if (manipulatorState == 2) {
        statusStr = "retreating";
      }
      
      Serial.println("status:" + statusStr + ",load:" + String(latestValidLoad, 2));
      
      lastPrnTime = currentTime;
    }

  } else {
    if (manualCommand == 1) {
      analogWrite(RPWM1, MANUAL_SPEED);
      analogWrite(LPWM1, 0);
      analogWrite(RPWM2, MANUAL_SPEED);
      analogWrite(LPWM2, 0);
      analogWrite(RPWM3, MANUAL_SPEED);
      analogWrite(LPWM3, 0);
    } else if (manualCommand == 2) {
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, MANUAL_SPEED);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, MANUAL_SPEED);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, MANUAL_SPEED);
    } else {
      analogWrite(RPWM1, 0);
      analogWrite(LPWM1, 0);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, 0);
      analogWrite(RPWM3, 0);
      analogWrite(LPWM3, 0);
    }
  }
} 