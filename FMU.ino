#include <Servo.h>

#define LED_PIN 13
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 4
#define MOTOR_3_PIN 5
#define MOTOR_4_PIN 6

#define CALIBRATE_ESC
#define THROTTLE_MIN 0
#define THROTTLE_MAX 179

boolean LED_STATUS = false;
int throttle = 0;
Servo motor[4];

void initializeESC()
{
    motor[0].attach(MOTOR_1_PIN);
    motor[1].attach(MOTOR_2_PIN);
    motor[2].attach(MOTOR_3_PIN);
    motor[3].attach(MOTOR_4_PIN);
}

void calibrateESC()
{
    throttle = THROTTLE_MAX;

    motor[0].write(throttle);
    motor[1].write(throttle);
    motor[2].write(throttle);
    motor[3].write(throttle);

    LED_STATUS = true;
    for (int i = 0; i < 15; i++) {
        digitalWrite(LED_PIN, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(500);
    }
    for (int i = 0; i < 25; i++) {
        digitalWrite(LED_PIN, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(100);
    }

    throttle = THROTTLE_MIN;
    motor[0].write(throttle);
    motor[1].write(throttle);
    motor[2].write(throttle);
    motor[3].write(throttle);
    
    LED_STATUS = true;
    for (int i = 0; i < 25; i++) {
        digitalWrite(LED_PIN, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(200);
    }
}

void setup()
{
    Serial.begin(38400);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, true);
    
    Serial.println("Initialize ESC");
    initializeESC();
    
    #ifdef CALIBRATE_ESC
        Serial.println("Calibrate ESC");
        calibrateESC();
    #endif
    
    LED_STATUS = true;
    digitalWrite(LED_PIN, LED_STATUS);
    Serial.println("ESC ready, engine start in 5s");
    delay(5000);
}

void loop()
{
    throttle = 30;
    motor[0].write(throttle);
    Serial.println(throttle, DEC);
    delay(15);
}

