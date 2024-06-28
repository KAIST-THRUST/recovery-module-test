#include <MPU9250.h>
#include <Servo.h>
#include "attitude.h"

#define SERVO_PWM 9
#define SAMPLE_RATE_HZ 40

#define COS45DEG 0.525322f
#define COS30DEG 0.866025f
#define COS35DEG 0.819152f

#define THRESHOLD_COS COS35DEG // change this as cos(trigger degree) as you want.

MPU9250 mpu;
Servo servo;
Attitude attitude;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  attitude.initAttitude();
  servo.attach(SERVO_PWM);
  servo.write(0); // to initial pos
  Serial.println("Boot Up Completed!");
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 1000/SAMPLE_RATE_HZ) {
      prev_ms = millis();
      readAndUpdateNewAttitude();
      triggerServo();
    }
  }

}

void readAndUpdateNewAttitude() {
  float new_quat[4];

  // get quaternion from MPU9250
  new_quat[0] = mpu.getQuaternionW();
  new_quat[1] = mpu.getQuaternionX();
  new_quat[2] = mpu.getQuaternionY();
  new_quat[3] = mpu.getQuaternionZ();

  // print out the new quaternion estimate
  Serial.print("q0: ");
  Serial.print(new_quat[0], 4);
  Serial.print(", q1: ");
  Serial.print(new_quat[1], 4);
  Serial.print(", q2: ");
  Serial.print(new_quat[2], 4);
  Serial.print(", q3: ");
  Serial.println(new_quat[3], 4);

  // update current attitude
  attitude.updateAttitude(new_quat);

  return;
}

void triggerServo() {
  // check current attitude satisfies trigger condition.
  Serial.println(attitude.isTilted(THRESHOLD_COS));
  if (attitude.isTilted(THRESHOLD_COS)) {
    servo.write(90); // triggered
  } else {
    servo.write(0); // not triggered
  }

  return;
}