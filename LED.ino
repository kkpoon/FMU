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
        case LED_Y:
            digitalWrite(LED_Y_PIN, value);
            break;
    }
}

void LED_COLOR(boolean Re, boolean Gr, boolean Bl, boolean Ye)
{
    digitalWrite(LED_R_PIN, Re);
    digitalWrite(LED_G_PIN, Gr);
    digitalWrite(LED_B_PIN, Bl);
    digitalWrite(LED_Y_PIN, Ye);
}

void blinkTimeout(int seconds, int freq)
{
    int cycle = seconds * freq * 2;
    int timeout = 1000 / freq / 2;

    boolean LED_STATUS = true;
    for (int i = 0; i < cycle; i++) {
        LED_COLOR(LED_STATUS, LED_STATUS, LED_STATUS, LED_STATUS);
        LED_STATUS = !LED_STATUS;
        delay(timeout);
    }
}
