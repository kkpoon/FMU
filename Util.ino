void LED(int16_t led, boolean value)
{
    switch (led) {
        case LED_R:
            digitalWrite(LED_R_PIN, value);
            break;
        case LED_G:
            digitalWrite(LED_G_PIN, value);
            break;
        case LED_B:
            digitalWrite(LED_B_PIN, value);
            break;
    }
}

void LED_COLOR(boolean R, boolean G, boolean B)
{
    digitalWrite(LED_R_PIN, R);
    digitalWrite(LED_G_PIN, G);
    digitalWrite(LED_B_PIN, B);
}

void blinkTimeout(int seconds, int freq)
{
    int cycle = seconds * freq * 2;
    int timeout = 1000 / freq / 2;

    boolean LED_STATUS = true;
    for (int i = 0; i < cycle; i++) {
        LED_COLOR(LED_STATUS, LED_STATUS, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(timeout);
    }
}

long complementary_filter(long current, long last, float factor)
{
    return current * factor + last * (1.0 - factor);
}

double complementary_filter(double current, double last, float factor)
{
    return current * factor + last * (1.0 - factor);
}
