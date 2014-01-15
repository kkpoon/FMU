#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

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

#endif
