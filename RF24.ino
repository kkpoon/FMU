#define RF24_TO_GROUND 0xF0F0F0F0E1LL
#define RF24_TO_FMU    0xF0F0F0F0D2LL

SensorData data;
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

void setupRF24()
{
    radio.begin();
    radio.setChannel(21);
    radio.setRetries(15,15);
    radio.setPayloadSize(sizeof(data));
    radio.openWritingPipe(RF24_TO_GROUND);
    radio.openReadingPipe(1, RF24_TO_FMU);
    radio.startListening();
}

void sendSensorData()
{
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
    data.mpuTemp = mpuTemp;

    radio.stopListening();

    bool ok = radio.write(&data, sizeof(data));

    if (ok)
        Serial.print("ok...");
    else
        Serial.print("failed.\n\r");

    radio.startListening();
}

