#include <ESP32Servo.h>
#include <ezButton.h>
#include "esp_timer.h"
// #include <Servo.h>

Servo servo;  // create servo object to control a servo

typedef enum SERVO_MODE {
  SERVO_CLOSE,
  SERVO_STOP_CLOSE,
  SERVO_OPEN,
  SERVO_STOP_OPEN
} ServoMode;

ServoMode servoMode = SERVO_STOP_OPEN;

#define MAX_PWM 2300
#define MID_PWM 1500
#define MIN_PWM 700

#define CLOSE_PWM 2300
#define HOLD_PWM 1500
#define OPEN_PWM 1350

#define BUTTON_PIN_1 25
#define BUTTON_PIN_2 26
#define SERVO_PIN 33

ezButton button1(BUTTON_PIN_1);
ezButton button2(BUTTON_PIN_2);

int lastState = HIGH; // the previous state from the input pin
int currentState;     // the current reading from the input pin

int currentPWM = 1500;

hw_timer_t *timer = NULL;

bool timerEnabled = false;

void IRAM_ATTR onTimer(){
  Serial.println("timer");
  changeServoState();
  timerStop(timer);
  timerEnabled = false;
}

void setup() {
  Serial.begin(115200);
  // pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  

  // button1.setDebounceTime(150);
  // button1.setDebounceTime(150);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, MIN_PWM, MAX_PWM); // attaches the servo on pin 9 to the servo object

  servoMode = SERVO_STOP_CLOSE;
  servo.writeMicroseconds(MID_PWM);


  timer = timerBegin(0, 80, true);
  timerStop(timer);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3 * 1000 * 1000, false);
  timerAlarmEnable(timer);

  // attachPin (33, double freq, uint8_t resolution_bits=10)
  // servo.write(0);   // rotate slowly servo to 0 degrees immediately
}

void changeServoState(){
  switch (servoMode){
    case (SERVO_CLOSE):
      Serial.println("SERVO_STOP_CLOSE");
      servoMode = SERVO_STOP_CLOSE;
      currentPWM = HOLD_PWM;
      // servo.writeMicroseconds(MID_PWM);
    break;
    case (SERVO_STOP_CLOSE):
      Serial.println("SERVO_OPEN");
      servoMode = SERVO_OPEN;
      currentPWM = CLOSE_PWM;
      // servo.writeMicroseconds(MAX_PWM);
    break;
    case (SERVO_OPEN):
      Serial.println("SERVO_STOP_OPEN");
      servoMode = SERVO_STOP_OPEN;
      currentPWM = HOLD_PWM;
      // servo.writeMicroseconds(MID_PWM);
    break;
    case (SERVO_STOP_OPEN):
      Serial.println("SERVO_CLOSE");
      servoMode = SERVO_CLOSE;
      currentPWM = OPEN_PWM;
      // servo.writeMicroseconds(MIN_2_PWM);
    break;
  }
  delay(200);
  return;
}

void loop() {
  button1.loop();
  button2.loop();
  // currentState = digitalRead(BUTTON_PIN_1);

  // if(lastState == LOW && currentState == HIGH){
  //   Serial.println("BUTTON PRESSED");
  //   changeServoState();
  // }
  // lastState = currentState;
  if (button1.isPressed()){
    changeServoState();
  }
  if (button2.isPressed()){
    Serial.println("BUTTON 2 PRESSED");

    timerRestart(timer);
    timerAlarmWrite(timer, 3 * 1000 * 1000, false);
    timerAlarmEnable(timer);
    timerStart(timer);
    timerEnabled = true;
    
  }
  servo.writeMicroseconds(currentPWM);
  // delay(200);
}