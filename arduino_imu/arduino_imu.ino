#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN_MPU 15  
#define LED_PIN 2 // 

bool blinkState = false;

//////////////////////////// MPU_1 control/status vars ///////////////////////////////////////////
bool dmpReady_MPU = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_MPU;   // holds actual interrupt status byte from MPU
uint8_t devStatus_MPU;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_MPU;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_MPU;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_MPU[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_MPU;           // [w, x, y, z]         quaternion container
VectorInt16 aa_MPU;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_MPU;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_MPU;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_MPU;    // [x, y, z]            gravity vector
float euler_MPU[3];         // [psi, theta, phi]    Euler angle container
float ypr_MPU[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_USED_MPU[3];      // [yaw, pitch, roll]   yaw/pitch/roll container for serial comms
float acc_USED_MPU[3];      // [x, y, z]            world-frame calculated acceleration for serial comms

volatile bool mpuInterrupt_MPU = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_MPU() {
    mpuInterrupt_MPU = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////

int c = 0;

union packed_float {
  float i;
  byte b[4];
} ax1,ay1,az1,gy1,gp1,gr1;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void sendCmd(float ypr1[], float acc1[], float ypr2[], float acc2[]);

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); 

    delay(500);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN_MPU, INPUT);
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(500);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_MPU = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    ///////////////////////////////////////////// Check for MPU /////////////////////////////////////////////
    // make sure it worked (returns 0 if so)
    if (devStatus_MPU == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP MPU..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_MPU));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady_MPU, RISING);
        mpuIntStatus_MPU = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_MPU = true;

        // get expected DMP packet size for later comparison
        packetSize_MPU = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP MPU_1 Initialization failed (code "));
        Serial.print(devStatus_MPU);
        Serial.println(F(")"));
    }
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  // MPU_1 Fetch //
  if (!dmpReady_MPU) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer_MPU)) {
    mpu.dmpGetQuaternion(&q_MPU, fifoBuffer_MPU);
    mpu.dmpGetAccel(&aa_MPU, fifoBuffer_MPU);
    mpu.dmpGetGravity(&gravity_MPU, &q_MPU);
    mpu.dmpGetYawPitchRoll(ypr_MPU, &q_MPU, &gravity_MPU);
    ypr_USED_MPU[0] = ypr_MPU[0] * 180/M_PI;
    ypr_USED_MPU[1] = ypr_MPU[1] * 180/M_PI;
    ypr_USED_MPU[2] = ypr_MPU[2] * 180/M_PI;
//     Serial.print("ypr\t");
//     Serial.print(ypr_USED_MPU[0]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU[1]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU[2]);
//     Serial.print("\t");
    mpu.dmpGetLinearAccel(&aaReal_MPU, &aa_MPU, &gravity_MPU);
    mpu.dmpGetLinearAccelInWorld(&aaWorld_MPU, &aaReal_MPU, &q_MPU);
    acc_USED_MPU[0] = ((float)aaWorld_MPU.x) / 16384.0;
    acc_USED_MPU[1] = ((float)aaWorld_MPU.y) / 16384.0;
    acc_USED_MPU[2] = ((float)aaWorld_MPU.z) / 16384.0;
//     Serial.print("aworld\t");
//     Serial.print(acc_USED_MPU[0]);
//     Serial.print("\t");
//     Serial.print(acc_USED_MPU[1]);
//     Serial.print("\t");
//     Serial.println(acc_USED_MPU[2]);
  }
 
  sendCmd(ypr_USED_MPU, acc_USED_MPU);
  delay(10);
  
}


void sendCmd(float ypr[], float acc[]) {
  ax1.i = acc[0];
  ay1.i = acc[1];
  az1.i = acc[2];
  gy1.i = ypr[0];
  gp1.i = ypr[1];
  gr1.i = ypr[2];

  const char buf[52] = {'#', 's', ax1.b[3], ax1.b[2], ax1.b[1], ax1.b[0],
                                  ay1.b[3], ay1.b[2], ay1.b[1], ay1.b[0],
                                  az1.b[3], az1.b[2], az1.b[1], az1.b[0],
                                  gy1.b[3], gy1.b[2], gy1.b[1], gy1.b[0],
                                  gp1.b[3], gp1.b[2], gp1.b[1], gp1.b[0],
                                  gr1.b[3], gr1.b[2], gr1.b[1], gr1.b[0],
                        '\r', '\n'
                       };

 for (uint8_t i = 0; i < 52; i++) {
   // Serial.write(cmd[i]);
    Serial.write(buf[i]);
  }                
}
