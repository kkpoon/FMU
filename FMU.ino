#include <Servo.h>
#include <Wire.h>
#include <Ping.h>
#include <UltraSonicProximity.h>
#include <KKMulticopterBoard.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#define MAX_DISTANCE 250
#define MAX_PLAY_TIME 60000

#define AIL_PIN      3
#define ELE_PIN      4
#define THR_PIN      5
#define RUD_PIN      6
#define TRIGGER_PIN  11
#define ECHO_PIN     12
#define LED_PIN      13

#define ALPHA 0.5
#define MPU6050_SENS_2G 16384
#define MPU6050_SENS_4G 8192
#define MPU6050_SENS_8G 4096
#define MPU6050_SENS_16G 2048
#define MPU6050_ACCEL_OFFSET_X -7361
#define MPU6050_ACCEL_OFFSET_Y 27753
#define MPU6050_ACCEL_OFFSET_Z 15597
#define MPU6050_GYRO_OFFSET_X -13
#define MPU6050_GYRO_OFFSET_Y -22
#define MPU6050_GYRO_OFFSET_Z 4

boolean LED_STATUS = false;

KKMulticopterBoard kkboard(AIL_PIN, ELE_PIN, THR_PIN, RUD_PIN);
UltraSonicProximity usProximity(TRIGGER_PIN, ECHO_PIN);

MPU6050 mpu6050;
// measured data
int16_t ac[3];
int16_t av[3];
int16_t sensor_temperature;
// calculated data
double g[3];
float pitch;
float roll;
float yaw;

int near_ground_height = 0;
int last_near_ground_height = 0;
double velocity[3];

int throttle;

elapsedMillis since;
elapsedMillis lastSensorRead;



void setupMPU()
{
    mpu6050.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    mpu6050.setXAccelOffset(0);
    mpu6050.setYAccelOffset(0);
    mpu6050.setZAccelOffset(0);

    mpu6050.setXGyroOffset(0);
    mpu6050.setYGyroOffset(0);
    mpu6050.setZGyroOffset(0);

    readSensors();
    adjustData();

    for (int i = 0; i < 3; i++) {
        g[i] = constrain(map(ac[i], 0, MPU6050_SENS_2G, 0, 1000), 0, 1000) / 1000.0;
    }
}

//void updateThrottle()
//{
//    thr.writeMicroseconds(throttle);
//}
//
//void holdAt(int distance_cm)
//{
//    int adjValue = 0;
//    if (near_ground_height < distance_cm) {
//        if (velocity[2] <= 0) {
//            throttle += map(distance_cm - near_ground_height, 1, 50, 10, 100);
//        }
//    } 
//    else if (near_ground_height > distance_cm) {
//        if (velocity[2] >= 0) {
//            throttle -= map(near_ground_height - distance_cm, 1, 50, 10, 100);
//        }
//    }
//    throttle += constrain(adjValue, 10, 100);
//    throttle = constrain(throttle, CONTROL_25P, CONTROL_75P);
//    updateThrottle();
//}
//
//void land(int seconds)
//{
//    int diff = throttle - CONTROL_MIN;
//    while(throttle > CONTROL_MIN) {
//        if (velocity[2] >= 0) {
//            throttle -= 100;
//        }
//        updateThrottle();
//        delay(seconds * 1000 / diff);
//    }
//}

void setup()
{
    Wire.begin();
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
    since = 0;
}

void loop()
{
    readSensors();
    adjustData();
    calculateData();

    lastSensorRead = 0;

    if (since > MAX_PLAY_TIME) {
        //land(10);
    } 
    else {
        //holdAt(30);
    }
    printData();
    delay(20);
}

//========== Sensor data functions ==========
void readSensors() {
    mpu6050.getMotion6(&ac[0], &ac[1], &ac[2], &av[0], &av[1], &av[2]);
    sensor_temperature = (mpu6050.getTemperature() + 12412) / 340;
    near_ground_height = usProximity.getDistanceInCM();
}

void adjustData() {
    ac[0] = ac[0] + MPU6050_ACCEL_OFFSET_X;
    ac[1] = ac[1] + MPU6050_ACCEL_OFFSET_Y;
    ac[2] = ac[2] + MPU6050_ACCEL_OFFSET_Z;
    av[0] = av[0] + MPU6050_GYRO_OFFSET_X;
    av[1] = av[1] + MPU6050_GYRO_OFFSET_Y;
    av[2] = av[2] + MPU6050_GYRO_OFFSET_Z;

    if (near_ground_height <= 0 || near_ground_height > MAX_DISTANCE) {
        near_ground_height = -1;
    }
}

void calculateData() {
    int norm = 0;
    for (int i = 0; i < 3; i++) {
        norm = map(ac[i], -MPU6050_SENS_2G, MPU6050_SENS_2G, -1000, 1000);
        norm = complementary_filter(norm, g[i] * 1000, ALPHA);
        g[i] = constrain(norm, -1000, 1000) / 1000.0;
    }

    pitch = -atan2(g[0], g[2]);
    roll = atan2(g[1], g[2]);

    if (near_ground_height > 0 && near_ground_height <= MAX_DISTANCE) {
        double v = (near_ground_height - last_near_ground_height) * 10.0 / lastSensorRead;
        velocity[2] = complementary_filter(v, velocity[2], ALPHA);
    } 
    else {
        velocity[2] = -343.2;
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
    Serial.print(ac[0]);
    Serial.print(",");
    Serial.print(ac[1]);
    Serial.print(",");
    Serial.print(ac[2]);
    Serial.print("    ");
    Serial.print(av[0]);
    Serial.print(",");
    Serial.print(av[1]);
    Serial.print(",");
    Serial.print(av[2]);
    Serial.print("    ");
    Serial.print(g[0]);
    Serial.print(",");
    Serial.print(g[1]);
    Serial.print(",");
    Serial.print(g[2]);
    Serial.print("    ");
    Serial.print(sensor_temperature);
    Serial.print("    ");
    Serial.print(near_ground_height / 58);
    Serial.print("    ");
    Serial.print(throttle);
    Serial.println("");
}

