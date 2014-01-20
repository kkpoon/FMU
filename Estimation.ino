
void estimateNearGroundVerticalVelocity() {
    if (near_ground_height > 0 && near_ground_height <= MAX_DISTANCE) {
        double v = (near_ground_height - last_near_ground_height) * 10.0 / (millis() - lastSensorRead);
        velocity[2] = complementary_filter(v, velocity[2], ALPHA);
    } 
    else {
        velocity[2] = -343.2;
    }
}
