void setupRF433()
{
    vw_set_tx_pin(RF433_TRAN_PIN);
    vw_setup(2000);
}

void sendData433()
{
    uint8_t carrier[16];
    carrier[0] = map(gg[0], INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[1] = map(gg[1], INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[2] = map(gg[2], INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[3] = map(stepUp(gravity.x, RF24_CARRIER_SCALE_GRAVITY), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[4] = map(stepUp(gravity.y, RF24_CARRIER_SCALE_GRAVITY), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[5] = map(stepUp(gravity.z, RF24_CARRIER_SCALE_GRAVITY), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[6] = map(aaWorld.x, INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[7] = map(aaWorld.y, INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[8] = map(aaWorld.z, INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[9] = map(stepUp(ypr[0], RF24_CARRIER_SCALE_YPR), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[10] = map(stepUp(ypr[1], RF24_CARRIER_SCALE_YPR), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[11] = map(stepUp(ypr[2], RF24_CARRIER_SCALE_YPR), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[12] = map(stepUp(q.w, RF24_CARRIER_SCALE_QUATERNION), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[13] = map(stepUp(q.x, RF24_CARRIER_SCALE_QUATERNION), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[14] = map(stepUp(q.y, RF24_CARRIER_SCALE_QUATERNION), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    carrier[15] = map(stepUp(q.z, RF24_CARRIER_SCALE_QUATERNION), INT16_MIN, INT16_MAX, 0, UINT8_MAX);
    vw_send(carrier, sizeof(carrier));
    vw_wait_tx();
}
