/*
   2024/12/28
   This is a demo for ESP8266 sending custom OSD messages to FC via MSP
*/
#include "MSP.h"
#include "osd_symbols.h"

MSP msp;
uint8_t cnt = 0;
uint8_t msg_length = 0;
char msg[MAX_CUSTOM_INFO_NUM][16] = {{0}};

void setup()
{
  Serial.begin(115200);
  msp.begin(Serial);
  delay(300);
}

void loop() {
  sprintf(msg[0], "%c %cTHIS", MSP2TEXT_CUSTOM_INFO, SYM_TOTAL_DISTANCE);
  sprintf(msg[1], "%c %c IS ", MSP2TEXT_CUSTOM_INFO + 1, SYM_SPEED);
  sprintf(msg[2], "%c %c  A ", MSP2TEXT_CUSTOM_INFO + 2, 0x24);
  sprintf(msg[3], "%c %cDEMO", MSP2TEXT_CUSTOM_INFO + 3, 0x18);
  sprintf(msg[4], "%c %cSHOW", MSP2TEXT_CUSTOM_INFO + 4, 0x19);
  sprintf(msg[5], "%c %cCUST", MSP2TEXT_CUSTOM_INFO + 5, 0x1A);
  sprintf(msg[6], "%c %cMSGS", MSP2TEXT_CUSTOM_INFO + 6, 0x1B);
  sprintf(msg[7], "%c %c%04d", MSP2TEXT_CUSTOM_INFO + 7, 0x1C, cnt);

  for (uint8_t msg_index = 0; msg_index <  MAX_CUSTOM_INFO_NUM; msg_index++) {
    msg_length = strlen(msg[msg_index]);
    msp.command2(MSP2_SET_TEXT, &msg[msg_index], msg_length, 0);
  }

  delay(1000);
  cnt++;
}
