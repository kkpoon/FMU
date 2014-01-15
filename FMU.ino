#include <Servo.h>
#include <SPI.h>
#include <Ping.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include <UltraSonicProximity.h>
#include <KKMulticopterBoard.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "SensorData.h"

// MPU6050 offset, calibrate by CalibrateMPU6050 project
#define MPU6050_ACCEL_OFFSET_X -773
#define MPU6050_ACCEL_OFFSET_Y 3278
#define MPU6050_ACCEL_OFFSET_Z 1660
#define MPU6050_GYRO_OFFSET_X -24
#define MPU6050_GYRO_OFFSET_Y -62
#define MPU6050_GYRO_OFFSET_Z -6

#define MAX_DISTANCE 250
#define MAX_PLAY_TIME 60000

#define RF24_TO_GROUND 0xF0F0F0F0E1LL
#define RF24_TO_FMU    0xF0F0F0F0D2LL

#define AIL_PIN         3
#define ELE_PIN         4
#define THR_PIN         5
#define RUD_PIN         6
#define TRIGGER_PIN     11
#define ECHO_PIN        12
#define LED_PIN         14
#define MPU6050_INT_PIN 20
#define RF24_CE_PIN     9
#define RF24_CSN_PIN    10

#define ALPHA 0.5

KKMulticopterBoard  kkboard(AIL_PIN, ELE_PIN, THR_PIN, RUD_PIN);
UltraSonicProximity usProximity(TRIGGER_PIN, ECHO_PIN);
RF24                radio(RF24_CE_PIN, RF24_CSN_PIN);
MPU6050             mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gg[3];          // [x, y, z]            gyro sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t sensor_temperature;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int near_ground_height = 0;
int last_near_ground_height = 0;
double velocity[3];

int throttle;

unsigned long since = millis();
unsigned long lastSensorRead = millis();
unsigned long lastPrint = millis();
unsigned long lastSendRF24 = millis();

void setupRF24()
{
    radio.begin();
    radio.setRetries(15,15);
    //radio.setPayloadSize(8);
    radio.openWritingPipe(RF24_TO_GROUND);
    radio.openReadingPipe(1, RF24_TO_FMU);
    radio.startListening();
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

void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    Serial.begin(38400);
    Serial.println("Setup RF24");
    setupRF24();
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);

    Serial.println("Setup MPU");
    if (!setupMPU()) {
        Serial.println("MPU6050 failed initialization");
        digitalWrite(LED_PIN, false);
        while (true) delay(100000);
    }

    Serial.println("Setup Flight Board");
    blinkTimeout(5, 10);
    digitalWrite(LED_PIN, false);
    kkboard.arm();
    blinkTimeout(5, 10);
    kkboard.idle();

    Serial.println("Ready");
    digitalWrite(LED_PIN, true);
    since = millis();
}

void loop()
{
    readSensors();
    calculateData();

    lastSensorRead = millis();

    if (millis() - since > MAX_PLAY_TIME) {
        //land(10);
    } 
    else {
        //holdAt(30);
    }
    if (millis() - lastPrint > 200) {
        //printData();
        lastPrint = millis();
    }
    if (millis() - lastSendRF24 > 5000) {
        sendSensorData();
        lastSendRF24 = millis();
    }
}

//========== Sensor data functions ==========
void readSensors() {
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
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }

    sensor_temperature = (mpu.getTemperature() + 12412) / 340;
    near_ground_height = 0;//usProximity.getDistanceInCM();
    if (near_ground_height <= 0 || near_ground_height > MAX_DISTANCE) {
        near_ground_height = -1;
    }
}

void calculateData() {
    if (near_ground_height > 0 && near_ground_height <= MAX_DISTANCE) {
        double v = (near_ground_height - last_near_ground_height) * 10.0 / (millis() - lastSensorRead);
        velocity[2] = complementary_filter(v, velocity[2], ALPHA);
    } 
    else {
        velocity[2] = -343.2;
    }
}

void sendSensorData()
{
    SensorData data;
    data.aa[0] = aa.x;
    data.aa[1] = aa.y;
    data.aa[2] = aa.z;
    data.gg[0] = gg[0];
    data.gg[1] = gg[1];
    data.gg[2] = gg[2];
    data.aaReal[0] = aaReal.x;
    data.aaReal[1] = aaReal.y;
    data.aaReal[2] = aaReal.z;
    data.aaWorld[0] = aaWorld.x;
    data.aaWorld[1] = aaWorld.y;
    data.aaWorld[2] = aaWorld.z;
    data.gravity[0] = gravity.x;
    data.gravity[1] = gravity.y;
    data.gravity[2] = gravity.z;
    data.euler[0] = euler[0];
    data.euler[1] = euler[1];
    data.euler[2] = euler[2];
    data.ypr[0] = ypr[0];
    data.ypr[1] = ypr[1];
    data.ypr[2] = ypr[2];
    data.quaternion[0] = q.w;
    data.quaternion[1] = q.x;
    data.quaternion[2] = q.y;
    data.quaternion[3] = q.z;
    data.mpuTemp = sensor_temperature;
    
    radio.stopListening();

    bool ok = radio.write(&data, sizeof(data));

    if (ok)
        Serial.print("ok...");
    else
        Serial.print("failed.\n\r");

    radio.startListening();
}

//========== Flight Control =========
void holdAt(int distance_cm)
{
    int adjValue = 0;
    if (near_ground_height < distance_cm) {
        if (velocity[2] <= 0) {
            throttle++;
        }
    } 
    else if (near_ground_height > distance_cm) {
        if (velocity[2] >= 0) {
            throttle--;
        }
    }
    throttle = constrain(throttle, 10, 100);
    kkboard.setThrottle(throttle);
}

void land()
{
    while(throttle > 0) {
        if (velocity[2] >= -5) {
            throttle--;
        }
        throttle = constrain(throttle, 0, 100);
        kkboard.setThrottle(throttle);
        delay(100);
    }
}

//========== Utility functions ==========
void blinkTimeout(int seconds, int freq)
{
    int cycle = seconds * freq * 2;
    int timeout = 1000 / freq / 2;

    boolean LED_STATUS = true;
    for (int i = 0; i < cycle; i++) {
        digitalWrite(LED_PIN, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(timeout);
    }
}

long complementary_filter(long current, long last, float factor)
{
    return current * factor + last * (1.0 - factor);
}

double complementary_filter(double current, double last, float factor)
{
    return current * factor + last * (1.0 - factor);
}




