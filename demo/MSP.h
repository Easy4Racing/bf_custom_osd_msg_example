/*
  MSP.h

  Copyright (c) 2017, Fabrizio Di Vittorio (fdivitto2013@gmail.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#pragma once

#include <Arduino.h>
#include <Stream.h>

#define MSP_STATUS               101
#define MSP_BOXIDS               119

#define MSP2_SET_TEXT                   0x3007

// MSP2_SET_TEXT variable types
#define MSP2TEXT_PILOT_NAME                      1
#define MSP2TEXT_CRAFT_NAME                      2
#define MSP2TEXT_PID_PROFILE_NAME                3
#define MSP2TEXT_RATE_PROFILE_NAME               4
#define MSP2TEXT_BUILDKEY                        5
#define MSP2TEXT_RELEASENAME                     6
#define MSP2TEXT_CUSTOM_INFO                	   7

// bits of getActiveModes() return value
#define MSP_MODE_ARM          0
#define MSP_MODE_ANGLE        1
#define MSP_MODE_HORIZON      2
#define MSP_MODE_NAVALTHOLD   3 /* cleanflight BARO */
#define MSP_MODE_MAG          4
#define MSP_MODE_HEADFREE     5
#define MSP_MODE_HEADADJ      6
#define MSP_MODE_CAMSTAB      7
#define MSP_MODE_NAVRTH       8 /* cleanflight GPSHOME */
#define MSP_MODE_NAVPOSHOLD   9 /* cleanflight GPSHOLD */
#define MSP_MODE_PASSTHRU    10
#define MSP_MODE_BEEPERON    11
#define MSP_MODE_LEDLOW      12
#define MSP_MODE_LLIGHTS     13
#define MSP_MODE_OSD         14
#define MSP_MODE_TELEMETRY   15
#define MSP_MODE_GTUNE       16
#define MSP_MODE_SONAR       17
#define MSP_MODE_BLACKBOX    18
#define MSP_MODE_FAILSAFE    19
#define MSP_MODE_NAVWP       20 /* cleanflight AIRMODE */
#define MSP_MODE_AIRMODE     21 /* cleanflight DISABLE3DSWITCH */
#define MSP_MODE_HOMERESET   22 /* cleanflight FPVANGLEMIX */
#define MSP_MODE_GCSNAV      23 /* cleanflight BLACKBOXERASE */
#define MSP_MODE_HEADINGLOCK 24
#define MSP_MODE_SURFACE     25
#define MSP_MODE_FLAPERON    26
#define MSP_MODE_TURNASSIST  27
#define MSP_MODE_NAVLAUNCH   28
#define MSP_MODE_AUTOTRIM    29


// MSP_API_VERSION reply
struct msp_api_version_t {
  uint8_t protocolVersion;
  uint8_t APIMajor;
  uint8_t APIMinor;
} __attribute__ ((packed));


// MSP_FC_VARIANT reply
struct msp_fc_variant_t {
  char flightControlIdentifier[4];
} __attribute__ ((packed));


// MSP_FC_VERSION reply
struct msp_fc_version_t {
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t versionPatchLevel;
} __attribute__ ((packed));


// MSP_BOARD_INFO reply
struct msp_board_info_t {
  char     boardIdentifier[4];
  uint16_t hardwareRevision;
} __attribute__ ((packed));


// MSP_BUILD_INFO reply
struct msp_build_info_t {
  char buildDate[11];
  char buildTime[8];
  char shortGitRevision[7];
} __attribute__ ((packed));


// MSP_RAW_IMU reply
struct msp_raw_imu_t {
  int16_t acc[3];  // x, y, z
  int16_t gyro[3]; // x, y, z
  int16_t mag[3];  // x, y, z
} __attribute__ ((packed));


// flags for msp_status_ex_t.sensor and msp_status_t.sensor
#define MSP_STATUS_SENSOR_ACC    1
#define MSP_STATUS_SENSOR_BARO   2
#define MSP_STATUS_SENSOR_MAG    4
#define MSP_STATUS_SENSOR_GPS    8
#define MSP_STATUS_SENSOR_SONAR 16


// MSP_STATUS_EX reply
struct msp_status_ex_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;
  uint16_t averageSystemLoadPercent;  // 0...100
  uint16_t armingFlags;
  uint8_t  accCalibrationAxisFlags;
} __attribute__ ((packed));


// MSP_STATUS
struct msp_status_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;
} __attribute__ ((packed));


// MSP_SENSOR_STATUS reply
struct msp_sensor_status_t {
  uint8_t isHardwareHealthy;  // 0...1
  uint8_t hwGyroStatus;
  uint8_t hwAccelerometerStatus;
  uint8_t hwCompassStatus;
  uint8_t hwBarometerStatus;
  uint8_t hwGPSStatus;
  uint8_t hwRangefinderStatus;
  uint8_t hwPitotmeterStatus;
  uint8_t hwOpticalFlowStatus;
} __attribute__ ((packed));


#define MSP_MAX_SUPPORTED_SERVOS 8

// MSP_SERVO reply
struct msp_servo_t {
  uint16_t servo[MSP_MAX_SUPPORTED_SERVOS];
} __attribute__ ((packed));


// MSP_SERVO_CONFIGURATIONS reply
struct msp_servo_configurations_t {
  __attribute__ ((packed)) struct {
    uint16_t min;
    uint16_t max;
    uint16_t middle;
    uint8_t rate;
    uint8_t angleAtMin;
    uint8_t angleAtMax;
    uint8_t forwardFromChannel;
    uint32_t reversedSources;
  } conf[MSP_MAX_SUPPORTED_SERVOS];
} __attribute__ ((packed));

#define MSP_MAX_SUPPORTED_MOTORS 8

// MSP_MOTOR reply
struct msp_motor_t {
  uint16_t motor[MSP_MAX_SUPPORTED_MOTORS];
} __attribute__ ((packed));


#define MSP_MAX_SUPPORTED_CHANNELS 16

// MSP_RC reply
struct msp_rc_t {
  uint16_t channelValue[MSP_MAX_SUPPORTED_CHANNELS];
} __attribute__ ((packed));


// MSP_ATTITUDE reply
struct msp_attitude_t {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
} __attribute__ ((packed));

// MSP_ANALOG reply
struct msp_analog_t {
  uint8_t  vbat;     // 0...255
  uint16_t mAhDrawn; // milliamp hours drawn from battery
  uint16_t rssi;     // 0..1023
  int16_t  amperage; // send amperage in 0.01 A steps, range is -320A to 320A
} __attribute__ ((packed));


// MSP_ARMING_CONFIG reply
struct msp_arming_config_t {
  uint8_t auto_disarm_delay;
  uint8_t disarm_kill_switch;
} __attribute__ ((packed));


// MSP_LOOP_TIME reply
struct msp_loop_time_t {
  uint16_t looptime;
} __attribute__ ((packed));


// MSP_RC_TUNING reply
struct msp_rc_tuning_t {
  uint8_t  rcRate8;  // no longer used
  uint8_t  rcExpo8;
  uint8_t  rates[3]; // R,P,Y
  uint8_t  dynThrPID;
  uint8_t  thrMid8;
  uint8_t  thrExpo8;
  uint16_t tpa_breakpoint;
  uint8_t  rcYawExpo8;
} __attribute__ ((packed));


// MSP_PID reply
struct msp_pid_t {
  uint8_t roll[3];     // 0=P, 1=I, 2=D
  uint8_t pitch[3];    // 0=P, 1=I, 2=D
  uint8_t yaw[3];      // 0=P, 1=I, 2=D
  uint8_t pos_z[3];    // 0=P, 1=I, 2=D
  uint8_t pos_xy[3];   // 0=P, 1=I, 2=D
  uint8_t vel_xy[3];   // 0=P, 1=I, 2=D
  uint8_t surface[3];  // 0=P, 1=I, 2=D
  uint8_t level[3];    // 0=P, 1=I, 2=D
  uint8_t heading[3];  // 0=P, 1=I, 2=D
  uint8_t vel_z[3];    // 0=P, 1=I, 2=D
} __attribute__ ((packed));


// MSP_MISC reply
struct msp_misc_t {
  uint16_t midrc;
  uint16_t minthrottle;
  uint16_t maxthrottle;
  uint16_t mincommand;
  uint16_t failsafe_throttle;
  uint8_t  gps_provider;
  uint8_t  gps_baudrate;
  uint8_t  gps_ubx_sbas;
  uint8_t  multiwiiCurrentMeterOutput;
  uint8_t  rssi_channel;
  uint8_t  dummy;
  uint16_t mag_declination;
  uint8_t  vbatscale;
  uint8_t  vbatmincellvoltage;
  uint8_t  vbatmaxcellvoltage;
  uint8_t  vbatwarningcellvoltage;
} __attribute__ ((packed));

// MSP_UID reply
struct msp_uid_t {
  uint32_t uid0;
  uint32_t uid1;
  uint32_t uid2;
} __attribute__ ((packed));

// MSP_FEATURE mask
#define MSP_FEATURE_RX_PPM              (1 <<  0)
#define MSP_FEATURE_VBAT                (1 <<  1)
#define MSP_FEATURE_UNUSED_1            (1 <<  2)
#define MSP_FEATURE_RX_SERIAL           (1 <<  3)
#define MSP_FEATURE_MOTOR_STOP          (1 <<  4)
#define MSP_FEATURE_SERVO_TILT          (1 <<  5)
#define MSP_FEATURE_SOFTSERIAL          (1 <<  6)
#define MSP_FEATURE_GPS                 (1 <<  7)
#define MSP_FEATURE_UNUSED_3            (1 <<  8)         // was FEATURE_FAILSAFE
#define MSP_FEATURE_UNUSED_4            (1 <<  9)         // was FEATURE_SONAR
#define MSP_FEATURE_TELEMETRY           (1 << 10)
#define MSP_FEATURE_CURRENT_METER       (1 << 11)
#define MSP_FEATURE_3D                  (1 << 12)
#define MSP_FEATURE_RX_PARALLEL_PWM     (1 << 13)
#define MSP_FEATURE_RX_MSP              (1 << 14)
#define MSP_FEATURE_RSSI_ADC            (1 << 15)
#define MSP_FEATURE_LED_STRIP           (1 << 16)
#define MSP_FEATURE_DASHBOARD           (1 << 17)
#define MSP_FEATURE_UNUSED_2            (1 << 18)
#define MSP_FEATURE_BLACKBOX            (1 << 19)
#define MSP_FEATURE_CHANNEL_FORWARDING  (1 << 20)
#define MSP_FEATURE_TRANSPONDER         (1 << 21)
#define MSP_FEATURE_AIRMODE             (1 << 22)
#define MSP_FEATURE_SUPEREXPO_RATES     (1 << 23)
#define MSP_FEATURE_VTX                 (1 << 24)
#define MSP_FEATURE_RX_SPI              (1 << 25)
#define MSP_FEATURE_SOFTSPI             (1 << 26)
#define MSP_FEATURE_PWM_SERVO_DRIVER    (1 << 27)
#define MSP_FEATURE_PWM_OUTPUT_ENABLE   (1 << 28)
#define MSP_FEATURE_OSD                 (1 << 29)


// MSP_FEATURE reply
struct msp_feature_t {
  uint32_t featureMask; // combination of MSP_FEATURE_XXX
} __attribute__ ((packed));


// MSP_BOARD_ALIGNMENT reply
struct msp_board_alignment_t {
  int16_t rollDeciDegrees;
  int16_t pitchDeciDegrees;
  int16_t yawDeciDegrees;
} __attribute__ ((packed));


// values for msp_current_meter_config_t.currentMeterType
#define MSP_CURRENT_SENSOR_NONE    0
#define MSP_CURRENT_SENSOR_ADC     1
#define MSP_CURRENT_SENSOR_VIRTUAL 2
#define MSP_CURRENT_SENSOR_MAX     CURRENT_SENSOR_VIRTUAL


// MSP_CURRENT_METER_CONFIG reply
struct msp_current_meter_config_t {
  int16_t currentMeterScale;
  int16_t currentMeterOffset;
  uint8_t currentMeterType; // MSP_CURRENT_SENSOR_XXX
  uint16_t batteryCapacity;
} __attribute__ ((packed));


// msp_rx_config_t.serialrx_provider
#define MSP_SERIALRX_SPEKTRUM1024      0
#define MSP_SERIALRX_SPEKTRUM2048      1
#define MSP_SERIALRX_SBUS              2
#define MSP_SERIALRX_SUMD              3
#define MSP_SERIALRX_SUMH              4
#define MSP_SERIALRX_XBUS_MODE_B       5
#define MSP_SERIALRX_XBUS_MODE_B_RJ01  6
#define MSP_SERIALRX_IBUS              7
#define MSP_SERIALRX_JETIEXBUS         8
#define MSP_SERIALRX_CRSF              9


// msp_rx_config_t.rx_spi_protocol values
#define MSP_SPI_PROT_NRF24RX_V202_250K 0
#define MSP_SPI_PROT_NRF24RX_V202_1M   1
#define MSP_SPI_PROT_NRF24RX_SYMA_X    2
#define MSP_SPI_PROT_NRF24RX_SYMA_X5C  3
#define MSP_SPI_PROT_NRF24RX_CX10      4
#define MSP_SPI_PROT_NRF24RX_CX10A     5
#define MSP_SPI_PROT_NRF24RX_H8_3D     6
#define MSP_SPI_PROT_NRF24RX_INAV      7


// MSP_RX_CONFIG reply
struct msp_rx_config_t {
  uint8_t   serialrx_provider;  // one of MSP_SERIALRX_XXX values
  uint16_t  maxcheck;
  uint16_t  midrc;
  uint16_t  mincheck;
  uint8_t   spektrum_sat_bind;
  uint16_t  rx_min_usec;
  uint16_t  rx_max_usec;
  uint8_t   dummy1;
  uint8_t   dummy2;
  uint16_t  dummy3;
  uint8_t   rx_spi_protocol;  // one of MSP_SPI_PROT_XXX values
  uint32_t  rx_spi_id;
  uint8_t   rx_spi_rf_channel_count;
} __attribute__ ((packed));


#define MSP_MAX_MAPPABLE_RX_INPUTS 8

// MSP_RX_MAP reply
struct msp_rx_map_t {
  uint8_t rxmap[MSP_MAX_MAPPABLE_RX_INPUTS];  // [0]=roll channel, [1]=pitch channel, [2]=yaw channel, [3]=throttle channel, [3+n]=aux n channel, etc...
} __attribute__ ((packed));


// values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
#define MSP_SENSOR_ALIGN_CW0_DEG        1
#define MSP_SENSOR_ALIGN_CW90_DEG       2
#define MSP_SENSOR_ALIGN_CW180_DEG      3
#define MSP_SENSOR_ALIGN_CW270_DEG      4
#define MSP_SENSOR_ALIGN_CW0_DEG_FLIP   5
#define MSP_SENSOR_ALIGN_CW90_DEG_FLIP  6
#define MSP_SENSOR_ALIGN_CW180_DEG_FLIP 7
#define MSP_SENSOR_ALIGN_CW270_DEG_FLIP 8

// MSP_SENSOR_ALIGNMENT reply
struct msp_sensor_alignment_t {
  uint8_t gyro_align;   // one of MSP_SENSOR_ALIGN_XXX
  uint8_t acc_align;    // one of MSP_SENSOR_ALIGN_XXX
  uint8_t mag_align;    // one of MSP_SENSOR_ALIGN_XXX
} __attribute__ ((packed));


// MSP_CALIBRATION_DATA reply
struct msp_calibration_data_t {
  int16_t accZeroX;
  int16_t accZeroY;
  int16_t accZeroZ;
  int16_t accGainX;
  int16_t accGainY;
  int16_t accGainZ;
  int16_t magZeroX;
  int16_t magZeroY;
  int16_t magZeroZ;
} __attribute__ ((packed));


// MSP_SET_HEAD command
struct msp_set_head_t {
  int16_t magHoldHeading; // degrees
} __attribute__ ((packed));


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

class MSP {

  public:

    void begin(Stream & stream, uint32_t timeout = 100);

    // low level functions

    uint8_t crc8_dvb_s2(uint8_t crc, byte a);

    void send(uint8_t messageID, void * payload, uint8_t size);

    void send2(uint16_t messageID, void * payload, uint8_t size);

    bool recv(uint8_t * messageID, void * payload, uint8_t maxSize, uint8_t * recvSize);

    bool recv2(uint16_t * messageID, void * payload, uint8_t maxSize, uint8_t * recvSize);

    bool waitFor(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize = NULL);

    bool waitFor2(uint16_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize = NULL);

    bool request(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize = NULL);

    bool command(uint8_t messageID, void * payload, uint8_t size, bool waitACK = true);

    bool command2(uint16_t messageID, void * payload, uint8_t size, bool waitACK = true);

    void reset();

    // high level functions

    bool getActiveModes(uint32_t * activeModes);


  private:

    Stream * _stream;
    uint32_t _timeout;

};
