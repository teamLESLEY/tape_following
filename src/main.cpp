#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "motor.hpp"
#include "tape.hpp"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define P_PIN PA5
#define D_PIN PA3
#define GAIN_PIN PA2
int lastError = 0;
unsigned long lastPositionStartTime = 0; // ms
unsigned long currentPositionStartTime = 0; // ms

#define PRINT_PIN PB14

TapeSensor ts(PA7, PA6, 150);
#define ERROR_ONE_OFF 2 // mm
#define ERROR_BOTH_OFF 10 // mm

Motor::DCMotor motorL(PB_9, PB_8, 900);
Motor::DCMotor motorR(PA_1, PA_0, 900);
#define MOTOR_BASE_SPEED 0.8

void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
}

void printPDParameters(bool running, int error, unsigned int kp,
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

void printPDCorrection(int error, double p, double d, double correction) {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("Motor on");
  display.printf("Error: %d\n\n\n\n", error);

  display.println("P*100 D*100 Corr*100");
  display.printf("%-5d %-5d %d", (int)(p * 100), (int)(d * 100), (int)(correction * 100));

  display.display();
}

int tapeSensorError(TapeSensor ts, int lastError) {
  bool tapeLeftOn = ts.isLeftOn();
  bool tapeRightOn = ts.isRightOn();

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
  pinMode(D_PIN, INPUT);
  pinMode(GAIN_PIN, INPUT);
  pinMode(PRINT_PIN, INPUT_PULLUP);
}

void loop() {
  unsigned int kp = analogRead(P_PIN);
  unsigned int kd = analogRead(D_PIN);
  unsigned int gain = analogRead(GAIN_PIN);

  int error = tapeSensorError(ts, lastError);

  // Calculate derivative
  unsigned long now = millis();
  if (error != lastError) {
    lastPositionStartTime = currentPositionStartTime;
    currentPositionStartTime = now;
  }
  double derivative = (error - lastError) / (now - lastPositionStartTime);

  double p = kp / 5000.0 * error;
  double d = kd / 50.0 * derivative;
  double correction = gain / 1024.0 * (p + d);

  double lSpeed = MOTOR_BASE_SPEED - correction;
  double rSpeed = MOTOR_BASE_SPEED + correction;

  if (!digitalRead(PRINT_PIN)) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    printPDParameters(false, error, kp, kd, gain);
  } else {
    motorL.setSpeed(constrain(lSpeed, -1, 1));
    motorR.setSpeed(constrain(rSpeed, -1, 1));
  }

  lastError = error;
}
