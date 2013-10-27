// @dir SalusControl

#include <JeeLib_raw.h>

#define SALUS_CMD_LEN_BYTES        4

uint8_t salus_addr123_boiler_on_buf[SALUS_CMD_LEN_BYTES]   = { 0x18, 0x01, 0x19, 0x5A };
uint8_t salus_addr123_boiler_off_buf[SALUS_CMD_LEN_BYTES] = { 0x18, 0x02, 0x1A, 0x5A };

void salusBoilerOnOff( uint8_t state )
{
    if (state)
        rf12_sendNow(&salus_addr123_boiler_on_buf, SALUS_CMD_LEN_BYTES);
    else
        rf12_sendNow(&salus_addr123_boiler_off_buf, SALUS_CMD_LEN_BYTES);
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[Salus]");
    
    rf12_initialize(1, RF12_868MHZ, 0xD4);
    rf12_control(RF12_DATA_RATE_2); // 2.4kbps
    rf12_control(0xA674); // 868.260khz
    rf12_control(0x9840); // 75khz freq shift
}

void loop() {  
      Serial.println("\nLoop");
      Serial.println("on");
      for (int i = 0; i < 30; i++)
      {
        salusBoilerOnOff(1);
        delay(30);
      }
      delay(60000);
      Serial.println("off");
      for (int i = 0; i < 30; i++)
      {
        salusBoilerOnOff(0);
        delay(30);
      }
      delay(60000);

//      Serial.println("off");
  //    for (int i = 0; i < 10; i++)
        //salusBoilerOnOff(0);
  
    //  delay(10000);
}
