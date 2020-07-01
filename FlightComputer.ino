#include <StaticThreadController.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


/// Variables and Definitions
#define elevatorPin 10
#define aileronPin 9
#define rudderPin 8
#define MPU6050_INTERRUPT_PIN 2

const int maximumServoInput = 70;
const int elevatorTrim = 89;
const int aileronTrim = 94;
const int rudderTrim  = 96;
const int MPU = 0x68; // MPU6050 I2C address


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container  
VectorFloat gravity;    // [x, y, z]            gravity vector
float elapsedTime, currentTime, previousTime;


Servo elevator;
Servo aileron;  
Servo rudder;


void driveServo(Servo servo, int trin ,int arg){
  servo.write(trin+ arg);
}

/// Class and Enumaration Declaritions

enum FlightStage{notarmed,armed,ascend,hold,enroute,descend,landing};
  
class FlightStatus{
  public:
    FlightStage flightstage;

};

class PlaneStatus {
 public:
    PlaneStatus(double y, double p, double r, double t) {
      yaw = y;
      pitch = p;
      roll = r;
      throttle = t;
    }// Access specifier
    double yaw, pitch, roll, throttle;        // Attribute (int variable)      
};

void manuer(PlaneStatus ms){
  ms.pitch;
}


class ManueverObject{
  public:
    int elevatorVal,aileronVal,rudderVal,throttleVal;  
    void executeManuever(){
      
        driveServo(elevator, elevatorTrim,elevatorVal);
        driveServo(aileron, aileronTrim,aileronVal);
        driveServo(rudder, rudderTrim,throttleVal);

        //Add throttle Control
      }
};


//static class PlaneStatusTemplates{

  //const PlaneStatus cruise(0,0,0,50);
  
//};


/// Objects


FlightStatus flightStatus;
PlaneStatus planeStatus(0,0,0,0);
//PlaneStatusTemplates planeTemplates;

ThreadController threadController = ThreadController();
Thread SensorThread = Thread();

MPU6050 mpu;

/// Control Related Functions
void resetServoPozition(){
    driveServo(elevator, elevatorTrim,0);
    driveServo(aileron, aileronTrim,0);
    driveServo(rudder, rudderTrim,0);
}



/// Sensor Related Functions
void readSensors(){
  readOrientation();
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU6050(){
 mpu.initialize();
 pinMode(MPU6050_INTERRUPT_PIN, INPUT);
 Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 int devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
  //      mpu.CalibrateAccel(6);
    //    mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void readOrientation(){
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            
           planeStatus.yaw = ypr[0];
           planeStatus.pitch = ypr[1];
           planeStatus.roll = ypr[2];
 
}
}

void setup() {
  Serial.begin(115200);

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    
    /// Objects Setup
    flightStatus.flightstage = notarmed;
    /// Controls Setup
    elevator.attach(elevatorPin);
    aileron.attach(aileronPin);
    rudder.attach(rudderPin);

    resetServoPozition();

    /// Sensor setup
    setupMPU6050();
    
    ///Thread Setup
    //SensorThread.setInterval(10); // Setts the wanted interval to be 10ms
    //SensorThread.onRun(readSensors); 
    //threadController.add(&SensorThread);

    

    
  delay(20);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  /// Use mainthread for readings
  /// fix in future
  readSensors();


}
