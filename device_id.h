#ifndef DEVICE_ID_H_INCLUDED
#define DEVICE_ID_H_INCLUDED

#define DEVICE_LOG_SIGNATURE 0x8E   /* ログファイル先頭のシグネチャ */
#define LOG_SIGNATURE_OLD 0x3E      /* ログファイルのデーターごとの先頭バイトシグネチャ (古い) */
#define LOG_SIGNATURE 0xAE          /* ログファイルのデーターごとの先頭バイトシグネチャ */
#define LOG_END_SIGNATURE 0xCE      /* ログデーター終了フラッグ */
#define TRANSMIT_START 0xA5
#define TRANSMIT_STOP  0xB5
#define TRANSMIT_HANDSHAKE 0xC5
#define TRANSMIT_HANDSHAKE_ACK 0xC6

typedef enum {
    ID_ADXL345,
    ID_LPS331AP,
    ID_LPS331AP_TEMP,
    ID_L3GD20,
    ID_HMC5883L,
    ID_MPU9150_GYRO,
    ID_MPU9150_ACC,
    ID_MPU9150_TEMP,
    ID_AK8975,
    ID_GPS,
    ID_LPS25H,
    DEVICE_COUNT,
} SensorDeviceId;

typedef enum {
    DEV_PRESS = 0x01,
    DEV_GYRO  = 0x02,
    DEV_MAG   = 0x04,
    DEV_ACC   = 0x08,
    DEV_TEMP  = 0x10,
    DEV_GPS   = 0x20,
    DEV_SD    = 0x40,
} Devices;

#endif
