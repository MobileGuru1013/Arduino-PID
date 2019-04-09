#include <TimerOne.h>

// Arduino
#define FAN 9
#define POT 0
#define T_SAMPLE 0.002

// PID constants
const float kp = 19.2;
const float ki = 200.0;
const float kd = 0.025;

// PID variables
int input = 0;
double angle = 0.0;
double output = 0.0;
double e = 0.0, e_prev = 0.0, sum_e = 0.0;
int maximum = 1, minimum = 0;
bool flag = false;

void control();

void setup() {
  pinMode(FAN, OUTPUT);
  pinMode(POT, INPUT);

  Serial.begin(9600);

  // Calibration
  Serial.print("Starting calibration... ");
  digitalWrite(LED_BUILTIN, HIGH);
  while (millis() < 5000) {
    angle = analogRead(POT);
    if (angle > maximum) maximum = angle;
    else if (angle < minimum) minimum = angle;
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("done. Maximum recorded value: ");
  Serial.println(maximum);

  // Setup TimerOne
  Timer1.initialize(2000); // interrupts every 2000us == 500Hz frequency
  Timer1.attachInterrupt(control);
  Timer1.pwm(FAN, output);
}

void loop() {
  while (true) {
    // Communication
    if (Serial.available() > 0) {
      input = Serial.parseInt(); // read user input from 0% to 100%
      if (input > 100) input = 100;
      else if (input < 0) input = 0;
      Serial.print("New input value: ");
      Serial.println(input);
    }

    // Control
    if (flag) {
      angle = analogRead(POT) * 100.0 / (maximum - minimum); // read position and convert to % of available range
      
      e_prev = e; // previous error
      e = input - angle ; // current error
      sum_e = sum_e + e; // sum of errors

      output = (kp * e + ki * T_SAMPLE * sum_e + kd * (e - e_prev) / T_SAMPLE) * 10.24;

      // Corrections
      if (output > 1024) {
        output = 1024;
        sum_e = (output / 10.24 - kd * (e - e_prev) / T_SAMPLE - kp * e) / (ki * T_SAMPLE);
      } else if (output < 0) {
        output = 0;
        sum_e = (output / 10.24 - kd * (e - e_prev) / T_SAMPLE - kp * e) / (ki * T_SAMPLE);
      }

      Timer1.setPwmDuty(FAN, output); // output from 0 to 1024
      flag = false;
    }
  }
}

void control() {
  flag = true;
}

