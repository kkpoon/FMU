// MPU6050 offset, calibrate by CalibrateMPU6050 project
#define MPU6050_ACCEL_OFFSET_X -773
#define MPU6050_ACCEL_OFFSET_Y 3278
#define MPU6050_ACCEL_OFFSET_Z 1660
#define MPU6050_GYRO_OFFSET_X  -24
#define MPU6050_GYRO_OFFSET_Y  -62
#define MPU6050_GYRO_OFFSET_Z  -6

MPU6050 mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int16_t mpuTemp;
Quaternion  q;          // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t     gg[3];      // [x, y, z]            gyro sensor measurements
float       euler[3];   // [psi, theta, phi]    Euler angle container
float       ypr[3];     // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void dmpDataReady() {
    mpuInterrupt = true;
}

boolean setupMPU()
{
    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }
    devStatus = mpu.dmpInitialize();

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        pinMode(MPU6050_INT_PIN, INPUT);
        attachInterrupt(MPU6050_INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        return false;
    }
    return true;
}

void readMPU()
{
    while (!mpuInterrupt && !mpu.dmpPacketAvailable());
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(gg, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetEuler(euler, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }
    mpuTemp = (mpu.getTemperature() + 12412) / 340;
}
