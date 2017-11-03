#include "Arduhdlc.h"
#define MAX_HDLC_FRAME_LENGTH 64
/* Function to send out byte/char */
void send_character(uint8_t data);

/* Function to handle a valid HDLC frame */
void hdlc_frame_handler(const uint8_t *data, uint16_t length);

/* Initialize Arduhdlc library with three parameters.
1. Character send function, to send out HDLC frame one byte at a time.
2. HDLC frame handler function for received frame.
3. Length of the longest frame used, to allocate buffer in memory */
Arduhdlc hdlc(&send_character, &hdlc_frame_handler, MAX_HDLC_FRAME_LENGTH);

/* Function to send out one 8bit character */
void send_character(uint8_t data)
{
    Serial.print((char)data);
}

uint8_t LED_STATE = HIGH;

/* Frame handler function. What to do with received data? */
void hdlc_frame_handler(const uint8_t *data, uint16_t length)
{
    // Do something with data that is in framebuffer
    LED_STATE = !LED_STATE;
    digitalWrite(15, LED_STATE);
    //delay(30);
    uint32_t num = *(uint32_t *)data;
    num++;
    hdlc.sendFrame((uint8_t *)&num, sizeof(num));
}

void setup()
{
    pinMode(15, OUTPUT);
    Serial.begin(115200);
}

void loop()
{
    while (Serial.available())
    {
        // get the new byte:
        char inChar = (char)Serial.read();
        // Pass all incoming data to hdlc char receiver
        hdlc.charReceiver(inChar);
    }
}
