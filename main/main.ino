#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#define LOOPTARGET 20
#define mpu_add 0x68  // MPU6050 주소
#define SERVO_PIN_1 9
#define SERVO_PIN_2 10
#define SERVO_PIN_3 11
#define SERVO_PIN_4 12
Servo servo1, servo2, servo3, servo4;
SoftwareSerial AT09(2, 3);  // RX, TX

class Kalman {
public:
  double getKalman(double acc, double gyro, double dt) {
    
    angle += dt * (gyro - bias);
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;
    
    double S = P[0][0] + R_measure;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = acc - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    
    double P_temp[2] = {P[0][0], P[0][1]};
    P[0][0] -= K[0] * P_temp[0];
    P[0][1] -= K[0] * P_temp[1];
    P[1][0] -= K[1] * P_temp[0];
    P[1][1] -= K[1] * P_temp[1];
    return angle;
  }

  void init(double qAngle, double qGyro, double rMeasure) {
    Q_angle = qAngle;
    Q_gyro = qGyro;
    R_measure = rMeasure;
    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
  }
  double getVar(int num) {
    switch (num) {
      case 0:
        return Q_angle;
        break;
      case 1:
        return Q_gyro;
        break;
      case 2:
        return R_measure;
        break;
    }
  }
private:
  double Q_angle, Q_gyro, R_measure;
  double angle, bias;
  double P[2][2], K[2];
};

/*class PIDcontrol {
public:
  void init(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    lastError = 0;
    integral = 0;
  }
  double calculate(double setpoint, double currentValue, double dt) {
    double error = setpoint - currentValue;
    integral += error * dt;
    double derivative = (error - lastError) / dt;
    lastError = error;
    return Kp * error + Ki * integral + Kd * derivative;
  }
private:

  double Kp, Ki, Kd;  double lastError;
  double integral;
};*/


Kalman kalmanRoll, kalmanPitch, kalmanYaw;
//PIDController pidRoll, pidPitch, pidYaw;
long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z;
double roll, pitch, yaw;
double dt;
double targetRoll = 0.0;
double targetPitch = 0.0;
double targetYaw = 0.0;
uint32_t pasttime;

void rocketInit() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  Wire.begin();
  Wire.beginTransmission(mpu_add);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  kalmanRoll.init(0.001, 0.002, 0.001);
  kalmanPitch.init(0.001, 0.002, 0.001);
  kalmanYaw.init(0.001, 0.002, 0.001);
  targetRoll = 0;
  targetPitch = 0;
  targetYaw = 0;
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);
  servo4.attach(SERVO_PIN_4);
}

void readIMU(){
  Wire.beginTransmission(mpu_add);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_add, 6, true);
  ac_x = Wire.read() << 8 | Wire.read();
  ac_y = Wire.read() << 8 | Wire.read();
  ac_z = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(mpu_add);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_add, 6, true);
  gy_x = Wire.read() << 8 | Wire.read();
  gy_y = Wire.read() << 8 | Wire.read();
  gy_z = Wire.read() << 8 | Wire.read();
}

void calcIMU() {
  roll = atan2(ac_y, ac_z) * RAD_TO_DEG;
  pitch = atan2(ac_x, sqrt(ac_y * ac_y + ac_z * ac_z)) * RAD_TO_DEG;
  yaw = atan2(gy_y, gy_x) * RAD_TO_DEG;

}

/*void PID {

}*/

void printSerial(){
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println();
}
unsigned long last = 20;
void setup() {
  rocketInit();
}

void loop() {
  if (millis() - pasttime > LOOPTARGET) {
    readIMU();
    calcIMU();
    printSerial();
    /*PID();
    writeRCS();
    readSensors();
    writeServo();
    teleRead();
    teleWrite();*/
    last = millis();
  }
}
