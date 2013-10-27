#include <JeelinkHaSalusRT500.h>
#include <JeelinkHaRF12ConfControl.h>

#define MAX_SERIAL_CMD_LEN  20

#define TRANSMITTER_CMD_LEN  3
static char TRANSMITTER_OK[TRANSMITTER_CMD_LEN] = {0x03,0x00,0x01};
static char TRANSMITTER_FAIL[TRANSMITTER_CMD_LEN] = {0x03,0x00,0x00};

#define CMD_HEADER_LEN 6

void send_transmitter_status( uint8_t state )
{
    if (state) Serial.write(TRANSMITTER_OK, TRANSMITTER_CMD_LEN);
    else Serial.write(TRANSMITTER_FAIL, TRANSMITTER_CMD_LEN);
}

void process_system_cmd( char *cmd )
{
  
}

void process_salus_rt500rf_cmd( char *cmd )
{
    char *payload = (cmd+CMD_HEADER_LEN);
    
    /* Payload format:
     * BYTE#  Description
     * 0      0x1 = On, 0x0 = Off
     */
    if (payload[0] == 0x1 || payload[0] == 0x0)
    {
      jeelinkha_salusrt500_onOff( (uint8_t)cmd[4] );
      send_transmitter_status(1); // OK
    } else send_transmitter_status(0); // FAIL
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
   * 2..4   Sensor ID
   * 5      Unit code
   * 6..N   Payload (byte N is (pkt_len - 1))
   */
  while (Serial.readBytes(p_cmd, 1) == 0);  
  Serial.readBytes((p_cmd+1), (int)cmd[0]);
  
  if (cmd[0] < CMD_HEADER_LEN+1) send_transmitter_status(0); // packets less than CMD_HEADER_LEN+1 bytes are meaningless as they have no payload
  else
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
