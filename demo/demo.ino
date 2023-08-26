/*
   v6.1:2023/8/23
   修改：
   CUSTOM_MSG_SPLITER从0x23改为0x00,
   msp.h->MSP2TEXT_CUSTOM_INFO=7
   用strncpy()取代springf()
   将msp.command2里面的sizeof(custommsg)改为custommsg的实际长度，因为第三行出现乱码
*/
#include "oltAuConfig.h"
#include "MSP.h"
#include "osd_symbols.h"

MSP msp;
void setup()
{
  Serial.begin(SERIAL_SPEED);
  msp.begin(Serial);
  delay(300);
}
uint8_t cnt = 0;
void loop() {
  char custommsg[30] = {0};
  uint8_t total_length = 0;

  custommsg[0] = MSP2TEXT_CUSTOM_INFO;
  custommsg[1] = ' ';

  custommsg[2] = SYM_HOMEFLAG;
  custommsg[3] = 'T';
  custommsg[4] = 'H';
  custommsg[5] = 'I';
  custommsg[6] = 'S';
  custommsg[7] = '\0';

  custommsg[8] = SYM_AH_LEFT;
  custommsg[9] = 'I';
  custommsg[10] = 'S';
  custommsg[11] = ' ';
  custommsg[12] = 'D';
  custommsg[13] = 'E';
  custommsg[14] = 'M';
  custommsg[15] = 'O';
  custommsg[16] = '\0';

  custommsg[17] = SYM_TOTAL_DISTANCE;
  custommsg[18] = ' ';
  custommsg[19] = cnt + '0';
  custommsg[20] = '\0';

  total_length = 21;
  msp.command2(MSP2_SET_TEXT, &custommsg, total_length, 0);

  delay(1000);
  cnt++;
}
