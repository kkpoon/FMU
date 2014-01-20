void print()
{
    Serial.print(mpu6050Data.motion.accel[0]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.accel[1]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.accel[2]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gyro[0]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gyro[1]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gyro[2]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gravity[0]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gravity[1]); Serial.print("\t");
    Serial.print(mpu6050Data.motion.gravity[2]); Serial.print("\t");
    Serial.print(mpu6050Data.accelReal[0]); Serial.print("\t");
    Serial.print(mpu6050Data.accelReal[1]); Serial.print("\t");
    Serial.print(mpu6050Data.accelReal[2]); Serial.print("\t");
    Serial.print(mpu6050Data.accelWorld[0]); Serial.print("\t");
    Serial.print(mpu6050Data.accelWorld[1]); Serial.print("\t");
    Serial.print(mpu6050Data.accelWorld[2]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.euler[0]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.euler[1]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.euler[2]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.ypr[0]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.ypr[1]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.ypr[2]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.quaternion[0]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.quaternion[1]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.quaternion[2]); Serial.print("\t");
    Serial.print(mpu6050Data.orientation.quaternion[3]); Serial.print("\t");
    Serial.print(mpu6050Data.temperature);
    Serial.println("");
}
