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
    carrier[0] = gg[0];
    carrier[1] = gg[1];
    carrier[2] = gg[2];
    carrier[3] = stepUp(gravity.x, RF24_CARRIER_SCALE_GRAVITY);
    carrier[4] = stepUp(gravity.y, RF24_CARRIER_SCALE_GRAVITY);
    carrier[5] = stepUp(gravity.z, RF24_CARRIER_SCALE_GRAVITY);
    carrier[6] = aaWorld.x;
    carrier[7] = aaWorld.y;
    carrier[8] = aaWorld.z;
    carrier[9] = stepUp(ypr[0], RF24_CARRIER_SCALE_YPR);
    carrier[10] = stepUp(ypr[1], RF24_CARRIER_SCALE_YPR);
    carrier[11] = stepUp(ypr[2], RF24_CARRIER_SCALE_YPR);
    carrier[12] = stepUp(q.w, RF24_CARRIER_SCALE_QUATERNION);
    carrier[13] = stepUp(q.x, RF24_CARRIER_SCALE_QUATERNION);
    carrier[14] = stepUp(q.y, RF24_CARRIER_SCALE_QUATERNION);
    carrier[15] = stepUp(q.z, RF24_CARRIER_SCALE_QUATERNION);
    radio.stopListening();
    boolean ok = radio.write(carrier, sizeof(carrier));
    if (ok) {
        Serial.println("ok");
    } 
    else {
        Serial.println("fail");
    }
    radio.startListening();
}


