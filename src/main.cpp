#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <Servo.h> 

// === J4 / J5 SERVO PIN KIOSZTÁSOK ===
#define SERVO_PIN 29          // J4: Csukló (SERVOS csatlakozó)
#define GRIPPER_SERVO_PIN 24  // J5: ÚJ ES08AII Fogó (RGB csatlakozó, GPIO 24)

// === J2 MOTOR PIN KIOSZTÁSOK (E) ===
#define E_STEP_PIN 14
#define E_DIR_PIN 13
#define E_ENABLE_PIN 15
#define E_DRIVER_ADDRESS 3
#define E_DIAG_PIN 16       

// === J1 MOTOR PIN KIOSZTÁSOK (X) ===
#define X_STEP_PIN 11       
#define X_DIR_PIN 10
#define X_ENABLE_PIN 12
#define X_DRIVER_ADDRESS 0
#define X_DIAG_PIN 4        

// === J3 MOTOR PIN KIOSZTÁSOK (Y) ===
#define Y_STEP_PIN 6        
#define Y_DIR_PIN 5         
#define Y_ENABLE_PIN 7      
#define Y_DRIVER_ADDRESS 2  
#define Y_DIAG_PIN 3        

#define R_SENSE 0.11f
#define STALL_VALUE 40      

#define STEPS_PER_360_DEG 48000
const int32_t BACKOFF_STEPS = 2000; 

const float STEPS_PER_DEGREE = 48000.0f / 360.0f;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperE = NULL; // J2
FastAccelStepper *stepperX = NULL; // J1
FastAccelStepper *stepperY = NULL; // J3

// Szervó objektumok és követő változók
Servo gripperServo; // J4 Csukló
Servo clawServo;    // J5 ÚJ Fogó

int servo_bazis_szog = 90;       // Pythonból érkező J4 alapérték
int utolso_kikuldott_szog = -1;  // Puffer a J4-nek

int claw_szog = 90;              // Pythonból érkező J5 ÚJ alapérték
int utolso_kikuldott_claw = -1;  // Puffer a J5-nek

TMC2209Stepper driverE(&Serial2, R_SENSE, E_DRIVER_ADDRESS);
TMC2209Stepper driverX(&Serial2, R_SENSE, X_DRIVER_ADDRESS);
TMC2209Stepper driverY(&Serial2, R_SENSE, Y_DRIVER_ADDRESS);

volatile bool e_elakadas_tortent = false;
volatile bool x_elakadas_tortent = false;
volatile bool y_elakadas_tortent = false; 

unsigned long e_indulas_ms = 0;
unsigned long x_indulas_ms = 0;
unsigned long y_indulas_ms = 0;

void __not_in_flash_func(e_stall_isr)() {
  if (millis() - e_indulas_ms > 500) { e_elakadas_tortent = true; }
}
void __not_in_flash_func(x_stall_isr)() {
  if (millis() - x_indulas_ms > 500) { x_elakadas_tortent = true; }
}
void __not_in_flash_func(y_stall_isr)() {
  if (millis() - y_indulas_ms > 500) { y_elakadas_tortent = true; }
}

void setup() {
  pinMode(E_ENABLE_PIN, OUTPUT);  digitalWrite(E_ENABLE_PIN, HIGH); 
  pinMode(X_ENABLE_PIN, OUTPUT);  digitalWrite(X_ENABLE_PIN, HIGH); 
  pinMode(Y_ENABLE_PIN, OUTPUT);  digitalWrite(Y_ENABLE_PIN, HIGH); 

  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  pinMode(E_DIAG_PIN, INPUT_PULLDOWN);
  pinMode(X_DIAG_PIN, INPUT_PULLDOWN);
  pinMode(Y_DIAG_PIN, INPUT_PULLDOWN); 

  Serial2.begin(115200);
  delay(200);

  driverE.begin(); driverE.toff(4); driverE.blank_time(24); driverE.I_scale_analog(false);
  driverE.rms_current(500); driverE.en_spreadCycle(false); driverE.microsteps(16);
  driverE.SGTHRS(STALL_VALUE); driverE.TCOOLTHRS(0xFFFFF); driverE.semin(0);

  driverX.begin(); driverX.toff(4); driverX.blank_time(24); driverX.I_scale_analog(false);
  driverX.rms_current(500); driverX.en_spreadCycle(false); driverX.microsteps(16);
  driverX.SGTHRS(STALL_VALUE); driverX.TCOOLTHRS(0xFFFFF); driverX.semin(0);

  driverY.begin(); driverY.toff(4); driverY.blank_time(24); driverY.I_scale_analog(false);
  driverY.rms_current(500); driverY.en_spreadCycle(false); driverY.microsteps(16);
  driverY.SGTHRS(STALL_VALUE); driverY.TCOOLTHRS(0xFFFFF); driverY.semin(0);
  delay(200);

  engine.init();
  stepperE = engine.stepperConnectToPin(E_STEP_PIN);
  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  
  if (!stepperE || !stepperX || !stepperY) { while(true); }

  stepperE->setDirectionPin(E_DIR_PIN, false); stepperE->setEnablePin(E_ENABLE_PIN, true);
  stepperE->setSpeedInHz(12000); stepperE->setAcceleration(15000);
  stepperX->setDirectionPin(X_DIR_PIN, false); stepperX->setEnablePin(X_ENABLE_PIN, true);
  stepperX->setSpeedInHz(12000); stepperX->setAcceleration(15000);
  stepperY->setDirectionPin(Y_DIR_PIN, true); stepperY->setEnablePin(Y_ENABLE_PIN, true);
  stepperY->setSpeedInHz(12000); stepperY->setAcceleration(15000);

  stepperE->enableOutputs(); stepperX->enableOutputs(); stepperY->enableOutputs();
  delay(200);

  gripperServo.attach(SERVO_PIN, 500, 2500); 
  gripperServo.write(servo_bazis_szog); 
  
  clawServo.attach(GRIPPER_SERVO_PIN, 500, 2500); 
  clawServo.write(claw_szog);
  delay(200);

  Serial.println("Rendszer kesz. Varom a Python csatlakozast es a Homing parancsot...");
}
void futtat_homing() {
  // FÁZIS 1: J3 (Y SLOT) ÖNÁLLÓ HOMING
  Serial.println("\n[J3 / Y] 1. Fazis: Biztonsagi tavolodas az utkozotol...");
  detachInterrupt(digitalPinToInterrupt(Y_DIAG_PIN)); 
  stepperY->moveTo(-BACKOFF_STEPS);
  while (stepperY->isRunning()) { delay(10); }
  delay(200); 
  Serial.println("[J3 / Y] 2. Fazis: Kereses az utkozo fele...");
  y_elakadas_tortent = false; y_indulas_ms = millis(); 
  attachInterrupt(digitalPinToInterrupt(Y_DIAG_PIN), y_stall_isr, RISING);
  stepperY->moveTo(STEPS_PER_360_DEG);
  while (!y_elakadas_tortent && stepperY->isRunning()) { delay(5); }
  detachInterrupt(digitalPinToInterrupt(Y_DIAG_PIN));
  int32_t y_stop_pos = stepperY->getPositionAfterCommandsCompleted();
  stepperY->forceStopAndNewPosition(y_stop_pos);
  delay(100);                      
  stepperY->moveTo(y_stop_pos - 400); 
  while (stepperY->isRunning()) { delay(10); }
  stepperY->setCurrentPosition(0);
  Serial.println("=== J3 HOMING KESZ ===");
  delay(500);

  // FÁZIS 2: J2 (E MOTOR) STANDARD HOMING
  Serial.println("\n[J2 / E] 1. Fazis: Biztonsagi tavolodas...");
  detachInterrupt(digitalPinToInterrupt(E_DIAG_PIN)); 
  stepperE->moveTo(-BACKOFF_STEPS);
  while (stepperE->isRunning()) { delay(10); }
  delay(200); 
  Serial.println("[J2 / E] 2. Fazis: Kereses az utkozo fele...");
  e_elakadas_tortent = false; e_indulas_ms = millis(); 
  attachInterrupt(digitalPinToInterrupt(E_DIAG_PIN), e_stall_isr, RISING);
  stepperE->moveTo(STEPS_PER_360_DEG);
  while (!e_elakadas_tortent && stepperE->isRunning()) { delay(5); }
  detachInterrupt(digitalPinToInterrupt(E_DIAG_PIN));
  int32_t e_stop_pos = stepperE->getPositionAfterCommandsCompleted();
  stepperE->forceStopAndNewPosition(e_stop_pos);
  delay(100);                      
  stepperE->moveTo(e_stop_pos - 400); 
  while (stepperE->isRunning()) { delay(10); }
  stepperE->setCurrentPosition(0);
  Serial.println("=== J2 HOMING KESZ ===");
  delay(500);

  // FÁZIS 3: J1 (X MOTOR) STANDARD HOMING
  Serial.println("\n[J1 / X] 1. Fazis: Biztonsagi tavolodas...");
  detachInterrupt(digitalPinToInterrupt(X_DIAG_PIN)); 
  stepperX->moveTo(-BACKOFF_STEPS);
  while (stepperX->isRunning()) { delay(10); }
  delay(200);
  Serial.println("\n[J1 / X] 2. Fazis: Kereses az utkozo fele...");
  x_elakadas_tortent = false; x_indulas_ms = millis(); 
  attachInterrupt(digitalPinToInterrupt(X_DIAG_PIN), x_stall_isr, RISING);
  stepperX->moveTo(STEPS_PER_360_DEG);
  while (!x_elakadas_tortent && stepperX->isRunning()) { delay(5); }
  detachInterrupt(digitalPinToInterrupt(X_DIAG_PIN));
  int32_t x_stop_pos = stepperX->getPositionAfterCommandsCompleted();
  stepperX->forceStopAndNewPosition(x_stop_pos);
  delay(100);
  stepperX->moveTo(x_stop_pos - 400);
  while (stepperX->isRunning()) { delay(10); }
  stepperX->setCurrentPosition(0);
  Serial.println("=== J1 HOMING KESZ ===");
  delay(500);

  // FÁZIS 4: VÉGLEGES CÉLRAÁLLÁS ÉS EZUTÁN ENNEK A NULLÁZÁSA
  Serial.println("\n=== Homing kesz, inditom a beallast a kezdo munkapozicioba... ===");
  stepperX->moveTo(-16266);        
  stepperE->moveTo(-3734);         
  stepperY->moveTo(-6131); 
  
  servo_bazis_szog = 90;
  gripperServo.write(servo_bazis_szog);
  utolso_kikuldott_szog = servo_bazis_szog;

  claw_szog = 90; 
  clawServo.write(claw_szog);
  utolso_kikuldott_claw = claw_szog;

  while (stepperX->isRunning() || stepperE->isRunning() || stepperY->isRunning()) { delay(10); }
  delay(200);

  stepperX->setCurrentPosition(0);
  stepperE->setCurrentPosition(0);
  stepperY->setCurrentPosition(0);
  Serial.println("STATUSZ: ALAPHELYZETBEN (EZ A NULLAPONT)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      if (input == "HOME") {
        futtat_homing();
      }
      else if (input.startsWith("MOVE ") || input.startsWith("QMOVE ")) {
        bool isQueue = input.startsWith("QMOVE ");
        int firstSpace = input.indexOf(' ');
        int secondSpace = input.indexOf(' ', firstSpace + 1);
        int thirdSpace = input.indexOf(' ', secondSpace + 1);
        int fourthSpace = input.indexOf(' ', thirdSpace + 1);
        int fifthSpace = input.indexOf(' ', fourthSpace + 1);

        if (firstSpace != -1 && secondSpace != -1 && thirdSpace != -1 && fourthSpace != -1) {
          long targetJ1 = input.substring(firstSpace + 1, secondSpace).toInt();
          long targetJ2 = input.substring(secondSpace + 1, thirdSpace).toInt();
          long targetJ3 = input.substring(thirdSpace + 1, fourthSpace).toInt();
          
          int targetJ4, targetJ5;
          int sixthSpace = (fifthSpace != -1) ? input.indexOf(' ', fifthSpace + 1) : -1;
          if (fifthSpace != -1) {
            targetJ4 = input.substring(fourthSpace + 1, fifthSpace).toInt();
            targetJ5 = (sixthSpace != -1) ? input.substring(fifthSpace + 1, sixthSpace).toInt()
                                          : input.substring(fifthSpace + 1).toInt();
          } else {
            targetJ4 = input.substring(fourthSpace + 1).toInt();
            targetJ5 = claw_szog; 
          }

          servo_bazis_szog = targetJ4;
          claw_szog = targetJ5; 

          // Motorsebességek: QMOVE ... j4 j5 v1 v2 v3 (lépés/s) -> a motorok együtt haladnak.
          // Sima MOVE (vagy sebesség nélküli QMOVE) esetén az alap 12000-es sebesség.
          uint32_t v1 = 12000, v2 = 12000, v3 = 12000;
          if (isQueue && sixthSpace != -1) {
            int seventhSpace = input.indexOf(' ', sixthSpace + 1);
            int eighthSpace = (seventhSpace != -1) ? input.indexOf(' ', seventhSpace + 1) : -1;
            if (seventhSpace != -1 && eighthSpace != -1) {
              v1 = constrain(input.substring(sixthSpace + 1, seventhSpace).toInt(), 10, 12000);
              v2 = constrain(input.substring(seventhSpace + 1, eighthSpace).toInt(), 10, 12000);
              v3 = constrain(input.substring(eighthSpace + 1).toInt(), 10, 12000);
            }
          }
          stepperX->setSpeedInHz(v1);
          stepperE->setSpeedInHz(v2);
          stepperY->setSpeedInHz(v3);

          stepperX->moveTo(targetJ1);
          stepperE->moveTo(targetJ2);
          stepperY->moveTo(targetJ3);

          if (!isQueue) {
            Serial.print("OK: MOVE -> J1:"); Serial.print(targetJ1);
            Serial.print(" J2:"); Serial.print(targetJ2);
            Serial.print(" J3:"); Serial.print(targetJ3);
            Serial.print(" J4:"); Serial.print(servo_bazis_szog);
            Serial.print(" J5:"); Serial.println(claw_szog);
          } else {
            Serial.println("SOR: OK");
          }
        }
      }
      else if (input == "QSTOP") {
        stepperX->stopMove(); stepperE->stopMove(); stepperY->stopMove();
        Serial.println("SOR: SIKERESEN TOROLVE ES MEGALLITVA");
      }
    }
  }

  // === VALÓS IDEJŰ HARDVERES VÍZSZINT-KOMPENZÁCIÓ ===
  int32_t j3_aktualis_lepes = stepperY->getCurrentPosition();
  float j3_aktualis_fok = (float)j3_aktualis_lepes / STEPS_PER_DEGREE;
  int korrigalt_j4 = servo_bazis_szog - (int)round(j3_aktualis_fok);

  if (korrigalt_j4 < 0) korrigalt_j4 = 0;
  if (korrigalt_j4 > 180) korrigalt_j4 = 180;

  if (korrigalt_j4 != utolso_kikuldott_szog) {
    gripperServo.write(korrigalt_j4);
    utolso_kikuldott_szog = korrigalt_j4;
  }

  // === J5 FOGÓ KÖZVETLEN VEZÉRLÉSE ===
  if (claw_szog < 0) claw_szog = 0;
  if (claw_szog > 180) claw_szog = 180;
  
  if (claw_szog != utolso_kikuldott_claw) {
    clawServo.write(claw_szog);
    utolso_kikuldott_claw = claw_szog;
  }
}