#define LOOPTARGET 20
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
