#include <Servo.h>
#include <Ping.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <UltraSonicProximity.h>
#include <KKMulticopterBoard.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define MAX_DISTANCE 250
#define MAX_PLAY_TIME 60000

#define AIL_PIN         3
#define ELE_PIN         4
#define THR_PIN         5
#define RUD_PIN         6
#define TRIGGER_PIN     11
#define ECHO_PIN        12
#define LED_PIN         13
#define MPU6050_INT_PIN 20

#define ALPHA 0.5

boolean LED_STATUS = false;

KKMulticopterBoard kkboard(AIL_PIN, ELE_PIN, THR_PIN, RUD_PIN);
UltraSonicProximity usProximity(TRIGGER_PIN, ECHO_PIN);

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
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
int32_t gg[3];       // [x, y, z]            gyro sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t sensor_temperature;

int near_ground_height = 0;
int last_near_ground_height = 0;
double velocity[3];

int throttle;

unsigned long since = millis();
unsigned long lastSensorRead = millis();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setupMPU()
{
    mpu.initialize();
    delay(1000);
    devStatus = mpu.dmpInitialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        pinMode(20, INPUT);
        attachInterrupt(MPU6050_INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        while(true) {
            blinkTimeout(1, 2);
        }
    }
}

void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(38400);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);

    setupMPU();
    blinkTimeout(5, 10);
    kkboard.arm();
    blinkTimeout(5, 10);
    kkboard.idle();

    LED_STATUS = true;
    digitalWrite(LED_PIN, LED_STATUS);
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
    //printData();
    delay(20);
}

//========== Sensor data functions ==========
void readSensors() {
    while (!mpuInterrupt && fifoCount < packetSize);
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
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(gg, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }
    
    sensor_temperature = (mpu.getTemperature() + 12412) / 340;
    near_ground_height = usProximity.getDistanceInCM();
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

    LED_STATUS = true;
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

void printData()
{
    Serial.print(aa.x);
    Serial.print(",");
    Serial.print(aa.y);
    Serial.print(",");
    Serial.print(aa.z);
    Serial.print("    ");
    Serial.print(gg[0]);
    Serial.print(",");
    Serial.print(gg[1]);
    Serial.print(",");
    Serial.print(gg[2]);
    Serial.print("    ");
    Serial.print(gravity.x);
    Serial.print(",");
    Serial.print(gravity.y);
    Serial.print(",");
    Serial.print(gravity.z);
    Serial.print("    ");
    Serial.print(sensor_temperature);
    Serial.print("    ");
    Serial.print(near_ground_height / 58);
    Serial.print("    ");
    Serial.print(throttle);
    Serial.println("");
}

