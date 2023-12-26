#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#define LOOPTARGET 20
#define mpu_add 0x68  // MPU6050
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

Kalman kalmanRoll, kalmanPitch, kalmanYaw;
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
  kalmanYaw.init(0.01, 0.002, 0.001);
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

void calcIMU(double ax, double ay, double az, double gx, double gy, double gz) {
  roll = atan2(ay, az) * RAD_TO_DEG;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  yaw = atan2(gy, gx) * RAD_TO_DEG;

}

void printSerial(){
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(yaw);
}  
unsigned long last = 0;
void setup() { 
  rocketInit();
}

void loop() {
  if(millis()-last > LOOPTARGET){
    readIMU();
    calcIMU();
    PID();
    writeRCS();
    readSensors();
    writeServo();
    teleRead();
    teleWrite();
    last = millis();
  }
}
