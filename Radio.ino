#define RF24_TO_GROUND 0xF0F0F0F0E1LL
#define RF24_TO_FMU    0xF0F0F0F0D2LL

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

void setupRF24()
{
    radio.begin();
    radio.enableDynamicPayloads();
    radio.setChannel(21);
    radio.setRetries(15,15);
    radio.openWritingPipe(RF24_TO_GROUND);
    radio.openReadingPipe(1, RF24_TO_FMU);
    radio.startListening();
}

void sendRF24YPR()
{
    int16_t carrier[4];
    carrier[0] = RF_CARRIER_TYPE_YPR;
    carrier[1] = stepUp(ypr[0], RF24_CARRIER_SCALE_YPR);
    carrier[2] = stepUp(ypr[1], RF24_CARRIER_SCALE_YPR);
    carrier[3] = stepUp(ypr[2], RF24_CARRIER_SCALE_YPR);
    radio.stopListening();
    boolean ok = false;
    ok = radio.write(carrier, sizeof(carrier));
    LED(LED_B, ok);
    radio.startListening();
}

void sendRF24PM25()
{
    int16_t carrier[2];
    carrier[0] = RF_CARRIER_TYPE_PM25;
    carrier[1] = int(round(pm_mgpm3 * 1000));
    radio.stopListening();
    boolean ok = false;
    ok = radio.write(carrier, sizeof(carrier));
    LED(LED_B, ok);
    radio.startListening();
}

void setupRF433()
{
    vw_set_tx_pin(RF433_TRAN_PIN);
    vw_setup(2000);
}

void sendRF433PM25()
{
    LED(LED_Y, true);
    uint8_t carrier[2];
    carrier[0] = RF_CARRIER_TYPE_PM25;
    carrier[1] = int(round(pm_mgpm3 * 1000));
    vw_send(carrier, sizeof(carrier));
    vw_wait_tx();
    LED(LED_Y, false);
}

