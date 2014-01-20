#define RF24_TO_GROUND 0xF0F0F0F0E1LL
#define RF24_TO_FMU    0xF0F0F0F0D2LL

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

void setupRF24()
{
    radio.begin();
    radio.enableDynamicPayloads();
    radio.setChannel(21);
    radio.setRetries(15,15);
    radio.openWritingPipe(RF24_TO_GROUND);
    radio.openReadingPipe(1, RF24_TO_FMU);
    radio.startListening();
}

void sendSensorData()
{
    int16_t carrier[16];
    carrier[0] = mpu6050Data.motion.gyro[0];
    carrier[1] = mpu6050Data.motion.gyro[1];
    carrier[2] = mpu6050Data.motion.gyro[2];
    carrier[3] = stepUp(mpu6050Data.motion.gravity[0], RF24_CARRIER_SCALE_GRAVITY);
    carrier[4] = stepUp(mpu6050Data.motion.gravity[1], RF24_CARRIER_SCALE_GRAVITY);
    carrier[5] = stepUp(mpu6050Data.motion.gravity[2], RF24_CARRIER_SCALE_GRAVITY);
    carrier[6] = mpu6050Data.accelWorld[0];
    carrier[7] = mpu6050Data.accelWorld[1];
    carrier[8] = mpu6050Data.accelWorld[2];
    carrier[9] = stepUp(mpu6050Data.orientation.ypr[0], RF24_CARRIER_SCALE_YPR);
    carrier[10] = stepUp(mpu6050Data.orientation.ypr[1], RF24_CARRIER_SCALE_YPR);
    carrier[11] = stepUp(mpu6050Data.orientation.ypr[2], RF24_CARRIER_SCALE_YPR);
    carrier[12] = stepUp(mpu6050Data.orientation.quaternion[0], RF24_CARRIER_SCALE_QUATERNION);
    carrier[13] = stepUp(mpu6050Data.orientation.quaternion[1], RF24_CARRIER_SCALE_QUATERNION);
    carrier[14] = stepUp(mpu6050Data.orientation.quaternion[2], RF24_CARRIER_SCALE_QUATERNION);
    carrier[15] = stepUp(mpu6050Data.orientation.quaternion[3], RF24_CARRIER_SCALE_QUATERNION);
    radio.stopListening();
//    boolean ok = radio.write(carrier, sizeof(carrier));
//    if (ok) {
//        Serial.println("ok");
//    } else {
//        Serial.println("fail");
//    }
    radio.startListening();
}

