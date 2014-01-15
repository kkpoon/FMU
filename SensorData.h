#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <Arduino.h>

typedef struct {
    int16_t aa[3];
    int16_t gg[3];
    int16_t aaReal[3];
    int16_t aaWorld[3];
    int16_t mpuTemp;
    float gravity[3];
    float euler[3];
    float ypr[3];
    float quaternion[4];
} 
SensorData;

void printSensorData(SensorData &data)
{
    Serial.print(data.aa[0]); 
    Serial.print("\t");
    Serial.print(data.aa[1]); 
    Serial.print("\t");
    Serial.print(data.aa[2]); 
    Serial.print("\t");
    Serial.print(data.gg[0]); 
    Serial.print("\t");
    Serial.print(data.gg[1]); 
    Serial.print("\t");
    Serial.print(data.gg[2]); 
    Serial.print("\t");
    Serial.print(data.gravity[0]); 
    Serial.print("\t");
    Serial.print(data.gravity[1]); 
    Serial.print("\t");
    Serial.print(data.gravity[2]); 
    Serial.print("\t");
    Serial.print(data.aaReal[0]); 
    Serial.print("\t");
    Serial.print(data.aaReal[1]); 
    Serial.print("\t");
    Serial.print(data.aaReal[2]); 
    Serial.print("\t");
    Serial.print(data.aaWorld[0]); 
    Serial.print("\t");
    Serial.print(data.aaWorld[1]); 
    Serial.print("\t");
    Serial.print(data.aaWorld[2]); 
    Serial.print("\t");
    Serial.print(degrees(data.euler[0])); 
    Serial.print("\t");
    Serial.print(degrees(data.euler[0])); 
    Serial.print("\t");
    Serial.print(degrees(data.euler[0])); 
    Serial.print("\t");
    Serial.print(degrees(data.ypr[0])); 
    Serial.print("\t");
    Serial.print(degrees(data.ypr[0])); 
    Serial.print("\t");
    Serial.print(degrees(data.ypr[0]));
    Serial.print("\t");
    Serial.print(data.quaternion[0]); 
    Serial.print("\t");
    Serial.print(data.quaternion[1]); 
    Serial.print("\t");
    Serial.print(data.quaternion[2]);
    Serial.print("\t");
    Serial.print(data.quaternion[3]);
    Serial.print("\t");
    Serial.print(data.mpuTemp);
    Serial.println("");
}

#endif


