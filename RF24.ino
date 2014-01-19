#define RF24_TO_GROUND 0xF0F0F0F0E1LL
#define RF24_TO_FMU    0xF0F0F0F0D2LL

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

void setupRF24()
{
    radio.begin();
    radio.setChannel(21);
    radio.setRetries(15,15);
    radio.enableDynamicPayloads();
    radio.openWritingPipe(RF24_TO_GROUND);
    radio.openReadingPipe(1, RF24_TO_FMU);
    radio.startListening();
}

void sendSensorData()
{
    RF24IntCarrier mpuTemp;
    RF24Int3DCarrier accel, gyro, accelReal, accelWorld;
    RF24Float3DCarrier gravity, euler, ypr;
    RF24QuaternionCarrier quaternion;
    
    accel.type = RF24_CARRIER_TYPE_ACCEL;
    accel.x = mpu6050Data.motion.accel[0];
    accel.y = mpu6050Data.motion.accel[1];
    accel.z = mpu6050Data.motion.accel[2];
    
    gyro.type = RF24_CARRIER_TYPE_GYRO;
    gyro.x = mpu6050Data.motion.gyro[0];
    gyro.y = mpu6050Data.motion.gyro[1];
    gyro.z = mpu6050Data.motion.gyro[2];
    
    gravity.type = RF24_CARRIER_TYPE_GRAVITY;
    gravity.x = mpu6050Data.motion.gravity[0];
    gravity.y = mpu6050Data.motion.gravity[1];
    gravity.z = mpu6050Data.motion.gravity[2];
    
    accelReal.type = RF24_CARRIER_TYPE_ACCEL_REAL;
    accelReal.x = mpu6050Data.accelReal[0];
    accelReal.y = mpu6050Data.accelReal[1];
    accelReal.z = mpu6050Data.accelReal[2];
    
    accelWorld.type = RF24_CARRIER_TYPE_ACCEL_WORLD;
    accelWorld.x = mpu6050Data.accelWorld[0];
    accelWorld.y = mpu6050Data.accelWorld[1];
    accelWorld.z = mpu6050Data.accelWorld[2];
    
    euler.type = RF24_CARRIER_TYPE_EULER;
    euler.x = mpu6050Data.orientation.euler[0];
    euler.y = mpu6050Data.orientation.euler[1];
    euler.z = mpu6050Data.orientation.euler[2];
    
    ypr.type = RF24_CARRIER_TYPE_YPR;
    ypr.x = mpu6050Data.orientation.ypr[0];
    ypr.y = mpu6050Data.orientation.ypr[1];
    ypr.z = mpu6050Data.orientation.ypr[2];
    
    quaternion.w = mpu6050Data.orientation.quaternion[0];
    quaternion.x = mpu6050Data.orientation.quaternion[1];
    quaternion.y = mpu6050Data.orientation.quaternion[2];
    quaternion.z = mpu6050Data.orientation.quaternion[3];
    
    mpuTemp.type = RF24_CARRIER_TYPE_TEMP;
    mpuTemp.value = mpu6050Data.temperature;

    radio.stopListening();
    radio.startWrite(&accel, RF24INT3DCARRIER_SIZE);
    radio.startWrite(&gyro, RF24INT3DCARRIER_SIZE);
    radio.startWrite(&accelReal, RF24INT3DCARRIER_SIZE);
    radio.startWrite(&accelWorld, RF24INT3DCARRIER_SIZE);
    radio.startWrite(&gravity, RF24FLOAT3DCARRIER_SIZE);
    radio.startWrite(&euler, RF24FLOAT3DCARRIER_SIZE);
    radio.startWrite(&ypr, RF24FLOAT3DCARRIER_SIZE);
    radio.startWrite(&quaternion, RF24QUATERNIONCARRIER_SIZE);
    radio.startWrite(&mpuTemp, RF24INTCARRIER_SIZE);
    radio.startListening();
}

