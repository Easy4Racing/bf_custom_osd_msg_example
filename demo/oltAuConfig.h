
//0.固件版本编号
String firmware_ver = "ESPLT_AU_a_6.1.17";
#define DEBUG_MODE 0

//1.板载LED相关配置
uint8_t LED = 13; //esp8285 = 13  esp8266 = 2

//2.按钮相关配置
#define BUTTON_INPUT  5
#define RACE_STATE  0
#define TEST_STATE  1
#define OTA_STATE   2
#define BIND_STATE  3
uint8_t work_state = 0;

//3.红外解码相关配置
#define ZERO 320
#define ONE  750
#define TOLERANCE 200
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))
#define TARGET_GATE_ID 31
#define IR_DECODE_MIN_GAP_TIME 2500

uint8_t RECV_PIN = 14;//红外接收头接GPIO14
boolean firstPass = true;
float pre_lap_time, this_lap_time, avg_lap_time = 0.0;
unsigned long now_sys_time, last_sys_time, passed_sys_time = 0;
uint16_t finished_lap = 0; //完成的圈数、最快圈

//4.MSP显示相关配置
#define SERIAL_SPEED 115200
#define MSP_DATA_SEND_GAP 3000  //MSP数据发送间隔=3000ms
#define CUSTOM_MSG_SPLITER '\0'
#define UNBOUND 0x20
#define BOUND 0x01

bool data_to_msp_flag = false;
unsigned long last_msp_send_time = 0;
//向MSP发送的数据
struct data_to_send_s {
  uint8_t data_type;        //0=对频通知,1=首次过门,2=非首次过门
  char au_mac[13];          //MAC地址
  
  int this_lap_cnt;         //当前圈
  unsigned long this_lap_time;  //当前圈速ms
  int best_lap;             //最快圈
  unsigned long best_lap_time;  //最快圈速ms
  unsigned long total_lap_time; //总时间ms
};
data_to_send_s data_to_send;

//5.OTA相关配置
const char* host_name = "Esp_LapTimer_AU";
const char* ssid = "HUAWEI-doudouzhu";
const char* password = "12345678";

//6.ESP-NOW通信相关配置
#define WIFI_CHANNEL 1
#define RESEND_MAX_TIMES 5
#define ESP_DATA_SEND_GAP 30
#define ESP_NOW_DATA_TYPE_BIND_NOTIFY 0
#define ESP_NOW_DATA_TYPE_RACE_START  1
#define ESP_NOW_DATA_TYPE_RACE_LAP    2
uint8_t ground_station_mac[6];
uint8_t ground_station_cnt = 0;
uint8_t esp_data_send_interval = 0;
unsigned long last_esp_now_send_time = 0;
bool au_gu_paired_flag = false;
bool data_to_ground_flag = false;
uint8_t resend_times = 0;//失败重发次数

//7.EEPROM参数
#define ALLOCAT_EEPROM_SIZE 30
#define EEPROM_OLD_DEVICE_FLAG_ADDRESS 0
#define EEPROM_MAC_START_ADDRESS 1
#define EEPROM_OLD_DEVICE_FLAG_CHAR 'O'
