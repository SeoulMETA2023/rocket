#include <Wire.h>

#define mpu_add 0x68  //mpu6050 address
#define A 8
#define B 9
#define C 10
#define D 11

#define kp 5
#define ki 0.1
#define kd 0.01
#define threshold 30

long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ;

double pitch_gy, yaw_gy;
double pitch_acc, yaw_acc;
double pitch_theta, yaw_theta;
double pitch_lastTheta, yaw_lastTheta;
double pitch_integralTheta, yaw_integralTheta;
double pitch_alpha, yaw_alpha;
double pitch_PID, yaw_PID;

double dt = 0.05;
uint32_t lastTime ;


void readIMU();
void calcIMU();
void PID();
void printSerial();
void RCS();
void tele();
void dataLog();

void setup() {
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);

  Serial.begin(9600) ;
  Wire.begin() ;
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
  Serial.println("Start");
}



void loop() {
  if(millis()-lastTime >= dt*1000){
    readIMU();
    calcIMU();
    printSerial();
    PID();
    RCS();
    tele();
    dataLog();
    lastTime = millis();
  }
}

void readIMU(){
  Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;
  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read() ;
  gy_y = Wire.read() << 8 | Wire.read() ;
  gy_z = Wire.read() << 8 | Wire.read() ;
}

void calcIMU(){
  pitch_acc = atan(ac_y/sqrt(ac_x*ac_x+ac_z*ac_z)) * 180 / PI ;  //acc data to degree data
  pitch_gy = gy_x/131. ;
  pitch_theta = 0.95*(pitch_theta+pitch_gy*dt)+0.05*(pitch_acc);

  yaw_acc = atan(ac_x/sqrt(ac_y*ac_y+ac_z*ac_z)) * 180 / PI ;  //acc data to degree data
  yaw_gy = gy_y/131. ;
  yaw_theta = 0.95*(yaw_theta+yaw_gy*dt)+0.05*(yaw_acc);
}

void PID(){ 
  pitch_integralTheta += dt * pitch_theta;
  pitch_alpha = (pitch_theta-pitch_lastTheta)/dt;
  pitch_PID = kp * pitch_theta + ki * pitch_integralTheta + kd * pitch_alpha;
  if(pitch_PID > 200) pitch_PID = 0;
  if(pitch_PID < -200) pitch_PID = 0;

  yaw_integralTheta += dt * yaw_theta;
  yaw_alpha = (yaw_theta-yaw_lastTheta)/dt;
  yaw_PID = kp * yaw_theta + ki * yaw_integralTheta + kd * yaw_alpha;
  if(yaw_PID > 200) yaw_PID = 0;
  if(yaw_PID < -200) yaw_PID = 0;
}

void RCS(){
  //Serial.println(pitch_PID);
  if(pitch_PID > threshold){
    digitalWrite(A,HIGH);
    digitalWrite(C,LOW);
    Serial.println("A");
  }
  else if(pitch_PID < -threshold){
    digitalWrite(C,HIGH);
    digitalWrite(A,LOW);
    Serial.println("B");
  }
  else{
    digitalWrite(C,LOW);
    digitalWrite(A,LOW);
  }


  if(yaw_PID > threshold){
    digitalWrite(B,HIGH);
    digitalWrite(D,LOW);
    Serial.println("A");
  }
  else if(yaw_PID < -threshold){
    digitalWrite(D,HIGH);
    digitalWrite(B,LOW);
    Serial.println("B");
  }
  else{
    digitalWrite(B,LOW);
    digitalWrite(D,LOW);
  }


}

void printSerial(){
  //Serial.print("pitch : ");
  //Serial.print(pitch_theta);
  //Serial.print("\t yaw : ");
  //Serial.println(yaw_theta);
}

void tele(){
  
}

void dataLog(){

}