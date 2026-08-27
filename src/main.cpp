#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <Servo.h> // RP2040 kompatibilis szervó könyvtár

// === J4 SERVO PIN KIOSZTÁS ===
#define SERVO_PIN 29       

// === J2 MOTOR PIN KIOSZTÁSOK (Az eredeti kódban E) ===
#define E_STEP_PIN 14
#define E_DIR_PIN 13
#define E_ENABLE_PIN 15
#define E_DRIVER_ADDRESS 3
#define E_DIAG_PIN 16       

// === J1 MOTOR PIN KIOSZTÁSOK (Az eredeti kódban X) ===
#define X_STEP_PIN 11       
#define X_DIR_PIN 10
#define X_ENABLE_PIN 12
#define X_DRIVER_ADDRESS 0
#define X_DIAG_PIN 4        

// === J3 MOTOR PIN KIOSZTÁSOK (Y slot, EZ INDUL LEGYELŐSZÖR A HOMINGBAN!) ===
#define Y_STEP_PIN 6        
#define Y_DIR_PIN 5         
#define Y_ENABLE_PIN 7      
#define Y_DRIVER_ADDRESS 2  // Igazolt valós UART cím
#define Y_DIAG_PIN 3        // GPIO 3 StallGuard végállás

#define R_SENSE 0.11f
#define STALL_VALUE 40      

#define STEPS_PER_360_DEG 48000
const int32_t BACKOFF_STEPS = 2000; 

// J3 40 fokos elmozdulásához szükséges lépésszám: (48000 / 360) * 40 = 5333 lépés
const int32_t J3_40_FOK_LEPES = -5333; 

// Átváltási arány: 48000 lépés = 360 fok -> 1 fok = 133.333 lépés
const float STEPS_PER_DEGREE = 48000.0f / 360.0f;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperE = NULL; // J2
FastAccelStepper *stepperX = NULL; // J1
FastAccelStepper *stepperY = NULL; // J3

// Szervó objektum és követő változók
Servo gripperServo; 
int servo_bazis_szog = 90;       // A Pythonból érkező kalibrációs alapérték
int utolso_kikuldott_szog = -1;  // Puffer a felesleges szervó-írások kiszűrésére

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
  // HARDVERES VÉDELEM: Azonnali lezárás (HIGH) az összes végfokra a boot pillanatában
  pinMode(E_ENABLE_PIN, OUTPUT);  digitalWrite(E_ENABLE_PIN, HIGH); 
  pinMode(X_ENABLE_PIN, OUTPUT);  digitalWrite(X_ENABLE_PIN, HIGH); 
  pinMode(Y_ENABLE_PIN, OUTPUT);  digitalWrite(Y_ENABLE_PIN, HIGH); 

  Serial.begin(115200);
  while (!Serial) { delay(10); } 
  
  Serial.println("\n=== BTT SKR PICO - VALÓS IDEJŰ HARDVERES KÖVETÉS ===");

  pinMode(E_DIAG_PIN, INPUT_PULLDOWN);
  pinMode(X_DIAG_PIN, INPUT_PULLDOWN);
  pinMode(Y_DIAG_PIN, INPUT_PULLDOWN); 

  Serial2.begin(115200);
  delay(200);

  // Driver inicializálások az UART buszon
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

  // FastAccelStepper motorok kapcsolódása
  engine.init();
  stepperE = engine.stepperConnectToPin(E_STEP_PIN);
  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  
  if (!stepperE || !stepperX || !stepperY) {
    Serial.println("Hiba: Nem sikerült a motorokhoz kapcsolódni!");
    while(true);
  }

  // Sebesség és irány alapbeállítások
  stepperE->setDirectionPin(E_DIR_PIN, false); stepperE->setEnablePin(E_ENABLE_PIN, true);
  stepperE->setSpeedInHz(12000); stepperE->setAcceleration(15000);

  stepperX->setDirectionPin(X_DIR_PIN, false); stepperX->setEnablePin(X_ENABLE_PIN, true);
  stepperX->setSpeedInHz(12000); stepperX->setAcceleration(15000);

  // J3 iránybeállítása a bevált inverz módban
  stepperY->setDirectionPin(Y_DIR_PIN, true); stepperY->setEnablePin(Y_ENABLE_PIN, true);
  stepperY->setSpeedInHz(12000); stepperY->setAcceleration(15000);

  // Kinyitjuk az összes végfok hidat
  stepperE->enableOutputs();
  stepperX->enableOutputs();
  stepperY->enableOutputs();
  delay(200);

  // Szervó felprogramozása és indító pozíció (középállás)
  gripperServo.attach(SERVO_PIN, 500, 2500); 
  gripperServo.write(servo_bazis_szog); 
  delay(200);

  Serial.println("Rendszer kesz. Varom a Python csatlakozast es a Homing parancsot...");
}
// A homing logika, ami a vegen le-nullazza a poziciokat
void futtat_homing() {
  // =======================================================
  // FÁZIS 1: J3 (Y SLOT) ÖNÁLLÓ HOMING
  // =======================================================
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

  Serial.println("[J3 / Y] 3. Fazis: Finom eltavolodas az utkozotol...");
  stepperY->moveTo(y_stop_pos - 400); 
  while (stepperY->isRunning()) { delay(10); }
  stepperY->setCurrentPosition(0);
  Serial.println("=== J3 HOMING KESZ ===");
  delay(500);

  // =======================================================
  // FÁZIS 2: J2 (E MOTOR) STANDARD HOMING
  // =======================================================
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

  Serial.println("[J2 / E] 3. Fazis: Finom eltavolodas...");
  stepperE->moveTo(e_stop_pos - 400); 
  while (stepperE->isRunning()) { delay(10); }
  stepperE->setCurrentPosition(0);
  Serial.println("=== J2 HOMING KESZ ===");
  delay(500);

  // =======================================================
  // FÁZIS 3: J1 (X MOTOR) STANDARD HOMING
  // =======================================================
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

  Serial.println("\n[J1 / X] 3. Fazis: Finom eltavolodas...");
  stepperX->moveTo(x_stop_pos - 400);
  while (stepperX->isRunning()) { delay(10); }
  stepperX->setCurrentPosition(0);
  Serial.println("=== J1 HOMING KESZ ===");
  delay(500);

  // =======================================================
  // FÁZIS 4: VÉGLEGES CÉLRAÁLLÁS ÉS EZUTÁN ENNEK A NULLÁZÁSA
  // =======================================================
  Serial.println("\n=== Homing kesz, inditom a beallast a kezdo munkapozicioba... ===");
  
  stepperX->moveTo(-16000);        
  stepperE->moveTo(-4000);         
  stepperY->moveTo(J3_40_FOK_LEPES); 
  
  // Szervó alapállapotba (90 fokra) húzása
  servo_bazis_szog = 90;
  gripperServo.write(servo_bazis_szog);
  utolso_kikuldott_szog = servo_bazis_szog;

  while (stepperX->isRunning() || stepperE->isRunning() || stepperY->isRunning()) { 
    delay(10); 
  }
  delay(200);

  // A felvett munkapozíció lesz az abszolút 0 pont!
  stepperX->setCurrentPosition(0);
  stepperE->setCurrentPosition(0);
  stepperY->setCurrentPosition(0);

  Serial.println("STATUSZ: ALAPHELYZETBEN (EZ A NULLAPONT)");
}

void loop() {
  // === 1. PARANCSOK FOGADÁSA ÉS FELDOLGOZÁSA ===
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      // --- HOME PARANCS ---
      if (input == "HOME") {
        Serial.println("Homing inditasa parancsra...");
        futtat_homing();
      }
      // --- AZONNALI MOVE PARANCS ---
      else if (input.startsWith("MOVE ")) {
        int firstSpace = input.indexOf(' ');
        int secondSpace = input.indexOf(' ', firstSpace + 1);
        int thirdSpace = input.indexOf(' ', secondSpace + 1);
        int fourthSpace = input.indexOf(' ', thirdSpace + 1);

        if (firstSpace != -1 && secondSpace != -1 && thirdSpace != -1 && fourthSpace != -1) {
          long targetJ1 = input.substring(firstSpace + 1, secondSpace).toInt();
          long targetJ2 = input.substring(secondSpace + 1, thirdSpace).toInt();
          long targetJ3 = input.substring(thirdSpace + 1, fourthSpace).toInt();
          int targetJ4  = input.substring(fourthSpace + 1).toInt();

          // A Pythonból érkező J4 most már CSAK a kalibrációs offszet (bázis) értéke!
          servo_bazis_szog = targetJ4;

          stepperX->moveTo(targetJ1);
          stepperE->moveTo(targetJ2);
          stepperY->moveTo(targetJ3);

          Serial.print("OK: Mozgas inditva -> J1:"); Serial.print(targetJ1);
          Serial.print(" J2:"); Serial.print(targetJ2);
          Serial.print(" J3:"); Serial.print(targetJ3);
          Serial.print(" J4 Bazis:"); Serial.println(servo_bazis_szog);
        } else {
          Serial.println("HIBA: Hibas MOVE formatum!");
        }
      }
      // --- QUEUE ASZINKRON MOZGÁS ETETÉS (QMOVE) ---
      else if (input.startsWith("QMOVE ")) {
        int firstSpace = input.indexOf(' ');
        int secondSpace = input.indexOf(' ', firstSpace + 1);
        int thirdSpace = input.indexOf(' ', secondSpace + 1);
        int fourthSpace = input.indexOf(' ', thirdSpace + 1);

        if (firstSpace != -1 && secondSpace != -1 && thirdSpace != -1 && fourthSpace != -1) {
          long targetJ1 = input.substring(firstSpace + 1, secondSpace).toInt();
          long targetJ2 = input.substring(secondSpace + 1, thirdSpace).toInt();
          long targetJ3 = input.substring(thirdSpace + 1, fourthSpace).toInt();
          int targetJ4  = input.substring(fourthSpace + 1).toInt();

          servo_bazis_szog = targetJ4;

          stepperX->moveTo(targetJ1);
          stepperE->moveTo(targetJ2);
          stepperY->moveTo(targetJ3);
          
          Serial.println("SOR: OK");
        } else {
          Serial.println("HIBA: Hibas QMOVE formatum!");
        }
      }
      // --- QUEUE AZONNALI TÖRLÉS ÉS VÉSZFÉKEZÉS (QSTOP) ---
      else if (input == "QSTOP") {
        stepperX->stopMove();
        stepperE->stopMove();
        stepperY->stopMove();
        Serial.println("SOR: SIKERESEN TOROLVE ES MEGALLITVA");
      }
    }
  }

  // === 2. VALÓS IDEJŰ HARDVERES VÍZSZINT-KOMPENZÁCIÓ ===
  // Lekérjük a J3 motor PILLANATNYI, ÉLŐ lépésszámát a futás közben
  int32_t j3_aktualis_lepes = stepperY->getCurrentPosition();

  // Átváltjuk a pillanatnyi lépést fizikai fokká
  float j3_aktualis_fok = (float)j3_aktualis_lepes / STEPS_PER_DEGREE;

  // Kiszámoljuk az élő kompenzációt (Mechanikai paralelogramma miatt: Alap - J3_fok)
  int korrigalt_j4 = servo_bazis_szog - (int)round(j3_aktualis_fok);

  // Szoftveres biztonsági korlát az ES08MDII-nek
  if (korrigalt_j4 < 0) korrigalt_j4 = 0;
  if (korrigalt_j4 > 180) korrigalt_j4 = 180;

  // Csak akkor írunk a szervóra, ha ténylegesen változott a szög (kíméli a szervót)
  if (korrigalt_j4 != utolso_kikuldott_szog) {
    gripperServo.write(korrigalt_j4);
    utolso_kikuldott_szog = korrigalt_j4;
  }
}
