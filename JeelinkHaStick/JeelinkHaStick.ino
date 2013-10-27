#include <JeelinkHaSalusRT500.h>
#include <JeelinkHaRF12ConfControl.h>

#define MAX_SERIAL_CMD_LEN  20

void process_system_cmd( char *cmd )
{
  
}

void process_salus_rt500rf_cmd( char *cmd )
{
    /* Payload format:
     * BYTE#  Description
     * 2      0x1 = On, 0x0 = Off
     */
    if (cmd[2] == 0x1 || cmd[2] == 0x0)
    {
      jeelinkha_salusrt500_onOff( (uint8_t)cmd[2] );
      Serial.println("OK");
    } else Serial.println("ERR");
}

void setup() {
    Serial.begin(57600);
    Serial.println("\nJeelinkHaStick v0.1");
    Serial.println("Salus RT500RF");
    
    jeelinkha_confctrl_init( SALUS_RT500RF_CFG );
}

void loop() {
  char cmd[MAX_SERIAL_CMD_LEN];
  char *p_cmd = (char *)&cmd;
  /* Serial format:
   * BYTE#  Description
   * 0      Pkt Len (include length byte)
   * 1      Command
   * 2..N   Payload (byte N is (pkt_len - 1))
   */
  while (Serial.readBytes(p_cmd, 1) == 0);
  Serial.readBytes((p_cmd+1), (int)cmd[0]);
  
  switch (cmd[1])
  {
     case 0x00:
       process_system_cmd(p_cmd);
       break;
     case 0x01:
       process_salus_rt500rf_cmd(p_cmd);
       break;
     default:
       break;
  }
}
