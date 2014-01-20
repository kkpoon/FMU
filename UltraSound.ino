UltraSonicProximity usProximity(TRIGGER_PIN, ECHO_PIN);

void readUltraSound()
{
    near_ground_height = 0;//usProximity.getDistanceInCM();
    if (near_ground_height <= 0 || near_ground_height > MAX_DISTANCE) {
        near_ground_height = -1;
    }
}
