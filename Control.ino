
KKMulticopterBoard  kkboard(AIL_PIN, ELE_PIN, THR_PIN, RUD_PIN);
int throttle;

void setupControl()
{
    kkboard.arm();
    delay(5000);
    kkboard.idle();
    delay(2000);
}

void holdAt(int distance_cm)
{
    int adjValue = 0;
    if (near_ground_height < distance_cm) {
        if (velocity[2] <= 0) {
            throttle++;
        }
    } 
    else if (near_ground_height > distance_cm) {
        if (velocity[2] >= 0) {
            throttle--;
        }
    }
    throttle = constrain(throttle, 10, 100);
    kkboard.setThrottle(throttle);
}

void land()
{
    while(throttle > 0) {
        if (velocity[2] >= -5) {
            throttle--;
        }
        throttle = constrain(throttle, 0, 100);
        kkboard.setThrottle(throttle);
        delay(100);
    }
}
