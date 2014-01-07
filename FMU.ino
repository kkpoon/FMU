#include <RCSwitch.h>
#include <Servo.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#define LED_PIN 13
#define RF_TRAN 14

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

#define MOTORS 4
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 4
#define MOTOR_3_PIN 20
#define MOTOR_4_PIN 21

//#define CALIBRATE_ESC
#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000

boolean LED_STATUS = false;

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

int throttle = 0;
int motor_throttle[MOTORS];
Servo motor[MOTORS];

RCSwitch transmitter = RCSwitch();

elapsedMillis since;
elapsedMillis sendDataInterval;

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

void readSensors() {
    mpu6050.getMotion6(&ac[0], &ac[1], &ac[2], &av[0], &av[1], &av[2]);
    sensor_temperature = (mpu6050.getTemperature() + 12412) / 340;
}

void adjustData() {
    ac[0] = ac[0] + MPU6050_ACCEL_OFFSET_X;
    ac[1] = ac[1] + MPU6050_ACCEL_OFFSET_Y;
    ac[2] = ac[2] + MPU6050_ACCEL_OFFSET_Z;
    av[0] = av[0] + MPU6050_GYRO_OFFSET_X;
    av[1] = av[1] + MPU6050_GYRO_OFFSET_Y;
    av[2] = av[2] + MPU6050_GYRO_OFFSET_Z;
}

void calculateData() {
    int norm = 0;
    for (int i = 0; i < 3; i++) {
        norm = map(ac[i], -MPU6050_SENS_2G, MPU6050_SENS_2G, -1000, 1000) * ALPHA + 
            g[i] * 1000 * (1.0 - ALPHA);
        g[i] = constrain(norm, -1000, 1000) / 1000.0;
    }

    pitch = -atan2(g[0], g[2]);
    roll = atan2(g[1], g[2]);
}

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

// ESC functions
void calibrateESC()
{
    for (int i = 0; i < MOTORS; i++) {
        motor[i].writeMicroseconds(THROTTLE_MAX);
    }

    blinkTimeout(5, 2);
    blinkTimeout(5, 10);
    
    for (int i = 0; i < MOTORS; i++) {
        motor[i].writeMicroseconds(THROTTLE_MIN);
    }
    
    blinkTimeout(10, 2);
}

void setupESC()
{
    Serial.println("Initialize ESC");
    motor[0].attach(MOTOR_1_PIN, THROTTLE_MIN, THROTTLE_MAX);
    motor[1].attach(MOTOR_2_PIN, THROTTLE_MIN, THROTTLE_MAX);
    motor[2].attach(MOTOR_3_PIN, THROTTLE_MIN, THROTTLE_MAX);
    motor[3].attach(MOTOR_4_PIN, THROTTLE_MIN, THROTTLE_MAX);
    
    #ifdef CALIBRATE_ESC
        Serial.println("Calibrate ESC");
        calibrateESC();
    #endif
    
    for (int i = 0; i < MOTORS; i++) {
        motor[i].writeMicroseconds(THROTTLE_MIN);
    }
    
    Serial.println("ESC ready in 5s");
    blinkTimeout(5, 2);
    Serial.println("ESC ready");
}

// TODO balance based on Accelerometer, now just for testing
void balance()
{
    motor_throttle[0] = motor_throttle[0] > 0 ? motor_throttle[0] - 1 : 0;
    motor_throttle[1] = motor_throttle[1] > 0 ? motor_throttle[1] - 1 : 0;
    motor_throttle[2] = motor_throttle[2] > 0 ? motor_throttle[2] - 1 : 0;
    motor_throttle[3] = motor_throttle[3] > 0 ? motor_throttle[3] - 1 : 0;
}

// TODO to be removed
void updateValue()
{
    if(Serial.available()) {
        int t = Serial.parseInt();
        if (t > 0) {
            for (int i = 0; i < MOTORS; i++) {
                motor_throttle[i] = t;
            }
        }
    }
}

void sendData()
{
    //transmitter.send("ax");
    transmitter.send(ac[0], 16);
    //transmitter.send("#");
}

void setup()
{
    Wire.begin();
    Serial.begin(38400);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);
    
    setupMPU();
    setupESC();

    transmitter.enableTransmit(RF_TRAN);
    
    LED_STATUS = true;
    digitalWrite(LED_PIN, LED_STATUS);
    since = 0;
    sendDataInterval = 0;
}

void loop()
{
    readSensors();
    adjustData();
    calculateData();
    
    if (since > 200) {
        //balance();
        since = 0;
    }
    if (sendDataInterval > 1000) {
        sendData();
        sendDataInterval = 0;
    }
    updateValue();
    for (int i = 0; i < MOTORS; i++) {
        motor[i].write(motor_throttle[i]);
    }
    
    for (int i = 0; i < MOTORS; i++) {
        Serial.print("Motor[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println(motor_throttle[i]);
    }
    
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
    Serial.println("");
    delay(20);
}

