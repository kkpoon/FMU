#include <Servo.h>
#include <SPI.h>
#include <Ping.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include "VirtualWire.h"
#include "UltraSonicProximity.h"
#include "KKMulticopterBoard.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RadioUtil.h"
#include "RunningMedian.h"

#define RF433_TRAN_PIN  2
#define AIL_PIN         3
#define ELE_PIN         4
#define THR_PIN         5
#define RUD_PIN         6
#define TRIGGER_PIN     7
#define ECHO_PIN        8
#define RF24_CE_PIN     9
#define RF24_CSN_PIN    10
#define LED_Y_PIN       14
#define LED_B_PIN       15
#define LED_G_PIN       16
#define LED_R_PIN       17
#define MPU6050_INT_PIN 20

#define ALPHA         0.5
#define MAX_DISTANCE  250
#define MAX_PLAY_TIME 60000

#define LED_R 1
#define LED_G 2
#define LED_B 3
#define LED_Y 4

int         near_ground_height = 0;
int         last_near_ground_height = 0;
double      velocity[3];

unsigned long since = millis();
unsigned long lastSensorRead = millis();
unsigned long lastPrint = millis();
unsigned long lastSendRF24 = millis();
unsigned long lastSendRF433 = millis();

void LED_COLOR(boolean Re, boolean Gr, boolean Bl, boolean Ye = false);

void setup()
{
    delay(5000);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    Serial.begin(38400);

    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    pinMode(LED_Y_PIN, OUTPUT);
    LED_COLOR(1,1,1,1);

    Serial.println("Setup RF24");
    setupRF24();

    Serial.println("Setup RF433");    
    setupRF433();

    Serial.println("Setup MPU");
    if (!setupMPU()) {
        Serial.println("MPU6050 failed initialization");
        LED_COLOR(1,0,0,0);
        while (true) delay(100000);
    }

    Serial.println("Setup Flight Board");
    setupControl();
    
    Serial.println("Setup PM25");
    setupPM25();

    Serial.println("Ready in 5 seconds");
    blinkTimeout(5, 2);
    LED_COLOR(0,1,0,0);
    since = millis();
}

void loop()
{
    readSensors();
    estimateNearGroundVerticalVelocity();

    lastSensorRead = millis();

    if (millis() - since > MAX_PLAY_TIME) {
        //land(10);
    } 
    else {
        //holdAt(30);
    }
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
    }
    if (millis() - lastSendRF24 > 100) {
        //sendRF24YPR();
        //sendRF24PM25();
        lastSendRF24 = millis();
    }
    if (millis() - lastSendRF433 > 50) {
        sendRF433PM25();
        lastSendRF433 = millis();
    }
    readPM25();
}

//========== Sensor data functions ==========
void readSensors() {
    readMPU();
    readUltraSound();
}


