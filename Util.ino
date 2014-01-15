void blinkTimeout(int seconds, int freq)
{
    int cycle = seconds * freq * 2;
    int timeout = 1000 / freq / 2;

    boolean LED_STATUS = true;
    for (int i = 0; i < cycle; i++) {
        digitalWrite(LED_PIN, LED_STATUS);
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
