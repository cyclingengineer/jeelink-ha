#include <JeelinkHaSalusRT500.h>
#include <JeelinkHaRF12ConfControl.h>
#include <stdarg.h>

#define MAX_SERIAL_CMD_LEN  20

/* helper defines */
/* Serial format:
 * BYTE#  Description
 * 0      Pkt Len (include length byte)
 * 1      Command
 * 2..4   Sensor ID
 * 5      Unit code
 * 6      Seq Nbr
 * 7..N   Payload (byte N is (pkt_len - 1))
 */
#define CMD_HEADER_LEN 7
#define PKT_LEN(a)        (a[0])
#define PKT_CMD(a)        (a[1])
#define PKT_SENSOR_ID(a)  ((a[2]<<16)|(a[3]<<8)|a[4])
#define pSENSOR_ID(a)     (a+2)
#define PKT_UNIT_CODE(a)  (a[5])
#define PKT_SEQ_NBR(a)    (a[6])
#define pPKT_PAYLOAD(a)   (a+CMD_HEADER_LEN)

//#define DEBUG
#ifdef DEBUG
void debug_print(char *fmt, ... ){
        char tmp[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, 128, fmt, args);
        va_end (args);
        Serial.print(tmp);
}
#else
void debug_print(char *fmt, ...) {
  /* do nothing */
}
#endif

#define STATUS_FAIL 0
#define STATUS_OK 1
#define STATUS_PKT_TOO_SHORT 2
void send_transmitter_status( uint8_t state, uint8_t seq_nbr )
{
    uint8_t data[8];//{0x8, 0x00, seq_nbr, state};
    data[0] = 0x8;
    data[1] = 0x0;
    data[2]=0;
    data[3]=0;
    data[4]=0;
    data[5]=0;
    data[6]=seq_nbr;
    data[7]=state;
    
    Serial.write((uint8_t*)data, 8);
}

void process_system_cmd( char *cmd )
{
  
}

void process_salus_rt500rf_cmd( char *cmd, uint8_t seq_nbr )
{
    char *payload = (char *)pPKT_PAYLOAD(cmd);

    /* Payload format:
     * BYTE#  Description
     * 0      0x1 = On, 0x0 = Off
     */  
    if (pPKT_PAYLOAD(cmd)[0] == 0x1 || pPKT_PAYLOAD(cmd)[0] == 0x0)
    {
      jeelinkha_salusrt500_onOff( pPKT_PAYLOAD(cmd)[0] );
      send_transmitter_status(1, seq_nbr); // OK
    } else send_transmitter_status(0, seq_nbr); // FAIL
}

void setup() {
    Serial.begin(57600);   
    jeelinkha_confctrl_init( SALUS_RT500RF_CFG );
}

void loop() {
  char cmd[MAX_SERIAL_CMD_LEN];
  char *p_cmd = (char *)cmd;

  while (Serial.readBytes(p_cmd, 1) == 0);  
  Serial.readBytes((p_cmd+1), (int)PKT_LEN(cmd));
     
  if (cmd[0] <= CMD_HEADER_LEN) send_transmitter_status(2, PKT_SEQ_NBR(cmd)); // packets less than CMD_HEADER_LEN+1 bytes are meaningless as they have no payload
  else
    switch (cmd[1])
    {
      case 0x00:
        process_system_cmd(p_cmd);
        break;
      case 0x01:
        process_salus_rt500rf_cmd(p_cmd, PKT_SEQ_NBR(cmd));
        break;
      default:
        break;
    }
}
