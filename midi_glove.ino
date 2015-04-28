#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define LED_PIN 13
int led_count = 0;
bool led_on = true;

//MPU Control
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//Motion variables
Quaternion q;
VectorInt16 accel;
VectorInt16 accelNoGrav;
VectorInt16 accelWorld;
VectorFloat gravity;
float ypr[3];
int pitchbend;
int cutoff;

volatile bool mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}

void setup() {
  pinMode(2, INPUT);
  attachInterrupt(2, dmpDataReady, RISING);
  Wire.begin();
  TWBR = 24;
  Serial.begin(115200);
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1527);
  
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println(F("DMP Init failed"));
  }
  pinMode(LED_PIN, OUTPUT);  
}

void loop() {
  if (!dmpReady) return;
  
  while(!mpuInterrupt && fifoCount < packetSize) {}
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&accel, fifoBuffer);
    mpu.dmpGetLinearAccel(&accelNoGrav, &accel, &gravity);
    
    pitchbend = 0x2000 + 0x2000 * 2 * (ypr[2]/HALF_PI);
    if(pitchbend > 0x3FFF) {
      pitchbend = 0x3FFF;
    }
    if(pitchbend < 0) {
      pitchbend = 0;
    }
    
    cutoff = 64 - 64 * (ypr[1]/HALF_PI);
    if(cutoff > 127) {
      cutoff = 127;
    }
    if(cutoff < 0) {
      cutoff = 0;
    }
    
    usbMIDI.sendPitchBend(pitchbend,1);
    usbMIDI.sendControlChange(74,cutoff,1);
    Serial.println(cutoff);
    
    led_count++;
    
    if (led_count >= 3){
      led_on = !led_on;
      digitalWrite(LED_PIN, led_on);
      led_count = 0;
    }
    
  }
          
//    Serial.print(ypr[0]);
//    Serial.print("\t");
//    Serial.print(ypr[1]);
//    Serial.print("\t");
//    Serial.print(ypr[2]);
//    Serial.print("\t");
//    Serial.print(accelNoGrav.x);
//    Serial.print("\t");}
//    Serial.print(accelNoGrav.y);
//    Serial.print("\t");
//    Serial.println(accelNoGrav.z);
}
