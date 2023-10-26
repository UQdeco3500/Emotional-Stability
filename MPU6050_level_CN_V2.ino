
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
/***********************MPU6050************************************/
#define angleMax 10     //倾斜角度
/***********************MPU6050************************************/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define mpuTime 50  //获取陀螺仪数据的间隔时间

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]       四元数容器
VectorInt16 aa;       // [x, y, z]          加速度传感器测量
VectorInt16 aaReal;   // [x, y, z]          无重力加速度传感器测量
VectorFloat gravity;  // [x, y, z]          重力矢量

float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int angle[3];

/******************************************************************/
/***********************串口打印************************************/
#define pintTime 50  //获取陀螺仪数据的间隔时间
//#define printangle   // 串口打印角度
#define printUnty  // 串口打印上传unity数据
/*****************************************************************/

void setup() {
  delay(1000);
  Serial.begin(9600);

  /***********************MPU6050初始化************************************/
  mpuInit();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  getMpuValue(mpuTime);
#ifdef printangle
  SerialPrint(pintTime);
#endif
#ifdef printUnty
  sendUnityData(pintTime);
#endif
}
void mpuInit() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // Serial.println(F("Testing device connections..."));
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Serial.print(F("DMP Initialization failed (code "));
    //  Serial.print(devStatus);
    //  Serial.println(F(")"));
  }
}
void getMpuValue(unsigned long time) {  //获取MPU6050
  static unsigned long lastTime = 0;    //建立静态局部变量，存储上次的时间值，系统断电后此值归零
  if (millis() - lastTime >= time) {    //判断当前时间-上次时间>=采样时间
    lastTime = millis();                //更新时间
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      angle[0] = ypr[1] * 180 / M_PI;
      angle[1] = ypr[2] * 180 / M_PI;
      angle[2] = ypr[0] * 180 / M_PI;

#endif
    }
  }
}

void SerialPrint(unsigned long time) {  //获取模拟值函数
  static unsigned long lastTime = 0;    //建立静态局部变量，存储上次的时间值，系统断电后此值归零
  if (millis() - lastTime >= time) {    //判断当前时间-上次时间>=采样时间
    lastTime = millis();                //更新时间
    Serial.print("angle\t");
    Serial.print(angle[0]);
    Serial.print("\t");
    Serial.print(angle[1]);
    Serial.print("\t");
    Serial.print(angle[2]);
    Serial.println();
  }
}

void sendUnityData(unsigned long _time) {
  static unsigned long lastTime = 0;   //建立静态局部变量，存储上次的时间值，系统断电后此值归零
  if (millis() - lastTime >= _time) {  //判断当前时间-上次时间>=采样时间
    lastTime = millis();               //更新时间
    if (abs(angle[0]) > angleMax) {
      if (angle[0] > 0) {
        Serial.println("I");
      } else {
        Serial.println("K");
      }
    }

    if (abs(angle[1]) > angleMax) {
      if (angle[1] > 0) {
        Serial.println("J");
      } else {
        Serial.println("L");
      }
    }

    if ( abs(angle[0]) < angleMax  && abs(angle[1]) < angleMax) {

        Serial.println("1");
      
    }
  }
}
