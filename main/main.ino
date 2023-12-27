#include <Wire.h>

#define mpu_add 0x68  //mpu6050 address
#define kp 1
#define ki 0.1
#define kd 0.01
#define threshold 20

class kalman {

  public :

    double getkalman(double acc, double gyro, double dt) {

      //project the state ahead

      angle += dt * (gyro - bias) ;
      //Project the error covariance ahead

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;

      P[0][1] -= dt * P[1][1] ;

      P[1][0] -= dt * P[1][1] ;

      P[1][1] += Q_gyro * dt ;



      //Compute the Kalman gain

      double S = P[0][0] + R_measure ;

      K[0] = P[0][0] / S ;

      K[1] = P[1][0] / S ;



      //Update estimate with measurement z

      double y = acc - angle ;

      angle += K[0] * y ;

      bias += K[1] * y ;



      //Update the error covariance

      double P_temp[2] = {P[0][0], P[0][1]} ;

      P[0][0] -= K[0] * P_temp[0] ;

      P[0][1] -= K[0] * P_temp[1] ;

      P[1][0] -= K[1] * P_temp[0] ;

      P[1][1] -= K[1] * P_temp[1] ;



      return angle ;

    } ;

    void init(double angle, double gyro, double measure) {

      Q_angle = angle ;

      Q_gyro = gyro ;

      R_measure = measure ;



      angle = 0 ;

      bias = 0 ;



      P[0][0] = 0 ;

      P[0][1] = 0 ;

      P[1][0] = 0 ;

      P[1][1] = 0 ;

    } ;

    double getvar(int num) {

      switch (num) {

        case 0 :

          return Q_angle ;

          break ;

        case 1 :

          return Q_gyro ;

          break ;

        case 2 :

          return R_measure ;

          break ;

      }

    } ;

  private :

    double Q_angle, Q_gyro, R_measure ;

    double angle, bias ;

    double P[2][2], K[2] ;

} ;


kalman kal ;

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

void setup() {
  Serial.begin(9600) ;
  Wire.begin() ;
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
  kal.init(0.001, 0.003, 0.03) ;  //init kalman filter
  Serial.println() ;
  Serial.print("parameter") ;
  Serial.print("\t") ;
  Serial.print(kal.getvar(0), 4) ;
  Serial.print("\t") ;
  Serial.print(kal.getvar(1), 4) ;
  Serial.print("\t") ;
  Serial.println(kal.getvar(2), 4) ;
}



void loop() {
  if(millis()-lastTime >= dt*1000){
    readIMU();
    calcIMU();
    printSerial();
    PID();
    RCS();
    tele();

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
  pitch_acc = atan2(ac_x, ac_z) * 180 / PI ;  //acc data to degree data
  pitch_gy = gy_y/131. ;
  pitch_theta = kal.getkalman(pitch_acc,pitch_gy,dt);

  yaw_acc = atan2(ac_y, ac_z) * 180 / PI ;  //acc data to degree data
  yaw_gy = gy_y/131. ;
  yaw_theta = kal.getkalman(yaw_acc,yaw_gy,dt);
}

void PID(){ 
  pitch_integralTheta += dt * pitch_theta;
  pitch_alpha = (pitch_theta-pitch_lastTheta)/dt;
  pitch_PID = kp * pitch_theta + ki * pitch_integralTheta + kd * pitch_alpha;

  yaw_integralTheta += dt * yaw_theta;
  yaw_alpha = (yaw_theta-yaw_lastTheta)/dt;
  yaw_PID = kp * yaw_theta + ki * yaw_integralTheta + kd * yaw_alpha;
}

void RCS(){
  
}

void printSerial(){
  Serial.print("kalman pitch") ;
  Serial.print("\t") ;
  Serial.print(pitch_theta) ;
  Serial.print("\t kalman yaw") ;
  Serial.print("\t") ;
  Serial.println(yaw_theta) ;
}

void tele(){

}