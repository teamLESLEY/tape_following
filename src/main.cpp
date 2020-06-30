#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "motor.hpp"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define P_PIN PA4
#define D_PIN PA3
#define GAIN_PIN PA2
int lastError = 0;
unsigned long lastPositionStartTime = 0; // ms
unsigned long currentPositionStartTime = 0; // ms

#define PRINT_PIN PB14
bool printSwitchedLastLoop = true;

#define TAPE_L PA7
#define TAPE_R PA6
#define TAPE_ON_MIN 150

#define ERROR_ONE_OFF 2 // mm
#define ERROR_BOTH_OFF 10 // mm

Motor::DCMotor motorL(PB_1, PB_0);
Motor::DCMotor motorR(PA_1, PA_0);
#define MOTOR_BASE_SPEED 0

void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
}

void printPDParameters(bool running, unsigned int error, unsigned int kp,
                       unsigned int kd, unsigned int gain) {
  display.clearDisplay();
  display.setCursor(0, 0);

  if (running) {
    display.println("Motor on");
  } else {
    display.println("Motor off");
  }
  display.printf("Error: %d\n\n\n\n", error);

  display.println("P    D    G");
  display.printf("%-4d %-4d %-4d", kp, kd, gain);

  display.display();
}

int tapeSensorError(unsigned int tapeLeftReading, unsigned int tapeRightReading, int lastError) {
  bool tapeLeftOn = tapeLeftReading > TAPE_ON_MIN;
  bool tapeRightOn = tapeRightReading > TAPE_ON_MIN;

  if (tapeLeftOn && tapeRightOn) {
    return 0;
  } else if (!tapeLeftOn && tapeRightOn) {
    return -ERROR_ONE_OFF;
  } else if (tapeLeftOn && !tapeRightOn) {
    return ERROR_ONE_OFF;
  } else {
    // In the fully off the tape case, assume it's off on the same
    // side as the last error. Implementation detail: if the robot
    // goes from on tape to fully off tape in one iteration, assume it
    // is on the right side of the tape.
    if (lastError < 0) {
      return -ERROR_BOTH_OFF;
    } else {
      return ERROR_BOTH_OFF;
    }
  }
}

void setup() {
  setupDisplay();

  pinMode(P_PIN, INPUT);
  pinMode(GAIN_PIN, INPUT);
  pinMode(D_PIN, INPUT);
  pinMode(PRINT_PIN, INPUT_PULLUP);

  pinMode(TAPE_L, INPUT);
  pinMode(TAPE_R, INPUT);
}

void loop() {
  unsigned int kp = analogRead(P_PIN);
  unsigned int kd = analogRead(D_PIN);
  unsigned int gain = analogRead(GAIN_PIN);

  unsigned int tapeLeft = analogRead(TAPE_L);
  unsigned int tapeRight = analogRead(TAPE_R);

  int error = tapeSensorError(tapeLeft, tapeRight, lastError);

  // Calculate derivative
  unsigned long now = millis();
  if (error != lastError) {
    lastPositionStartTime = currentPositionStartTime;
    currentPositionStartTime = now;
  }
  double derivative = (error - lastError) / (now - lastPositionStartTime);

  if (!digitalRead(PRINT_PIN)) {
    printSwitchedLastLoop = true;
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    printPDParameters(false, error, kp, kd, gain);
  } else {
    if (printSwitchedLastLoop) {
      printSwitchedLastLoop = false;

      printPDParameters(true, error, kp, kd, gain);
    }

    int p = kp * error;
    int d = kd * derivative;
    double correction = gain / 100000.0 * (p + d);

    double lSpeed = MOTOR_BASE_SPEED - correction;
    double rSpeed = MOTOR_BASE_SPEED + correction;
    motorL.setSpeed(constrain(lSpeed, -1, 1));
    motorR.setSpeed(constrain(rSpeed, -1, 1));
  }

  lastError = error;
}
