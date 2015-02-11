/*
 * LPS25Hコントロールライブラリ
 *
 * ブロッキング動作
 * SLA : SDO=0で5C
 *
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480
 *
 */

#ifndef LPS25H_H_INCLUDED
#define LPS25H_H_INCLUDED

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>

typedef enum LPS25HTempAvg_tag {
    LPS25HTempAvg8   = 0x00,
    LPS25HTempAvg16  = 0x04,
    LPS25HTempAvg32  = 0x08,
    LPS25HTempAvg64  = 0x0C,
} LPS25HTempAvg;

typedef enum LPS25HPresAvg_tag {
    LPS25HPresAvg8   = 0x00,
    LPS25HPresAvg32  = 0x01,
    LPS25HPresAvg128 = 0x02,
    LPS25HPresAvg512 = 0x03,
} LPS25HPresAvg;

typedef enum LPS25HDataRate_tag {
    LPS25HOneShot         = 0x00,
    LPS25H1_1Hz           = 0x10,
    LPS25H7_7Hz           = 0x20,
    LPS25H12dot5_12dot5Hz = 0x30,
    LPS25H25_25Hz         = 0x40,
} LPS25HDataRate;

typedef struct LPS25HUnit_tag {
    uint8_t address;
    int32_t pressure;
    int16_t temp;
    uint8_t ctrl_1;
} LPS25HUnit;

#ifdef __cplusplus
extern "C" {
#endif

char lps25h_init( LPS25HUnit *unit, uint8_t address, LPS25HDataRate rate, LPS25HPresAvg pres_avg, LPS25HTempAvg temp_avg );
char lps25h_start( LPS25HUnit *unit );
char lps25h_stop( LPS25HUnit *unit );
char lps25h_one_shot( LPS25HUnit *unit );
char lps25h_data_ready( LPS25HUnit *unit );
char lps25h_temp_data_ready( LPS25HUnit *unit );
char lps25h_read( LPS25HUnit *unit );
char lps25h_read_temp( LPS25HUnit *unit );

#ifdef __cplusplus
}
#endif

#endif
