#include <AccelStepper.h>
#include <HCSR04.h>
#include <LCD_I2C.h>
#include <Wire.h>


#define TRIGGER_PIN 5
#define ECHO_PIN 6
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

enum SystemState { DEMARRAGE,
                   MESURE_DISTANCE,
                   AFFICHAGE };
SystemState currentState = DEMARRAGE;

const int maxDegreeA = 360;
const int lowestValue = 1;
const int maxValue = 2038.0;
const int maxDegreeB = 170.0;
const int minDegreeB = 10.0;
float MaxStep = maxDegreeB * (maxValue / maxDegreeA);
float MinStep = minDegreeB * (maxValue / maxDegreeA);
float StepBack = 0;
float currentDistance = 0;
int currentDegree = 10;
bool ouvrir = false;

unsigned long lastDisplayTime = 0;
unsigned long lastDistanceTime = 0;
unsigned long lastSerialTime = 0;
const unsigned long DISPLAY_INTERVAL = 100;
const unsigned long DISTANCE_INTERVAL = 50;
const unsigned long SERIAL_INTERVAL = 100;


void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  myStepper.setMaxSpeed(800);
  myStepper.setAcceleration(120);
  myStepper.setSpeed(200);
  displayStartup();
}

void displayStartup() {
  lcd.setCursor(0, 0);
  lcd.print("Labo 4A");
  lcd.setCursor(0, 1);
  lcd.print("etd:2412876");
  lastDisplayTime = millis();
}

void measureDistance() {
  float distance = hc.dist();
  if (distance > 0) {
    currentDistance = distance;
    if (currentDistance < 30 && ouvrir == false) {
      ouvrir = true;
      controlDoor();
    }
  }
}

void controlDoor() {
  if (ouvrir) {
    myStepper.enableOutputs();
    myStepper.moveTo(MaxStep);
    StepBack = MaxStep;
  }
  currentDegree = DegreeToSteps(myStepper.currentPosition());
  currentDegree = stepsToDegree(currentDegree);
  ouvrir = false;
}


void lcdDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(currentDistance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);

  if (currentDistance > 60) {
    lcd.print("Porte: ");
    lcd.print("Fermer");
  } else if (currentDistance < 30) {
    if (myStepper.currentPosition() == MaxStep) {
      lcd.print("Porte: ");
      lcd.print("Ouverte");
    } else {
      lcd.print("deg:");
      lcd.print(currentDegree);
      lcd.print(" degree");
    }
  }
}

void serialDisplay() {
  Serial.print("etd:2412876,dist:");
  Serial.print(currentDistance);
  Serial.print(",deg:");
  Serial.println(currentDegree);
}


void loop() {

  unsigned long currentTime = millis();
  myStepper.run();
  if (myStepper.distanceToGo() == 0) {
    if (currentDistance > 60) {
      myStepper.moveTo(-StepBack);
      currentDegree = myStepper.currentPosition();
      currentDegree = DegreeToSteps(currentDegree);
      currentDegree = stepsToDegree(currentDegree);
    }
    myStepper.disableOutputs();
  }
  StepBack = 0.0;

  switch (currentState) {
    case DEMARRAGE:
      if (currentTime - lastDisplayTime >= 2000) {
        currentState = MESURE_DISTANCE;
        lcd.clear();
      }
      break;

    case MESURE_DISTANCE:
      if (currentTime - lastDistanceTime >= DISTANCE_INTERVAL) {
        lastDistanceTime = currentTime;
        measureDistance();
        currentState = AFFICHAGE;
      }
      break;

    case AFFICHAGE:
      if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = currentTime;
        lcdDisplay();
      }
      if (currentTime - lastSerialTime >= SERIAL_INTERVAL) {
        lastSerialTime = currentTime;
        serialDisplay();
      }
      currentState = MESURE_DISTANCE;
      break;
  }
}




float DegreeToSteps(float steps) {

  return map(steps, lowestValue, maxValue, MinStep, MaxStep);
}
float stepsToDegree(float steps) {
  return map(steps, MinStep, MaxStep, minDegreeB, maxDegreeB);
}