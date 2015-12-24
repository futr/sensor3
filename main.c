/*
 * sensor3用 main.c
 *
 */

#include "ide.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "sd.h"
#include "lps25h.h"
#include "ak8975.h"
#include "mpu9150.h"
#include "micomfs.h"
#include "usart.h"
#include "device_id.h"

#define LED_STATUS    _BV( PD7 )
#define SW_START_STOP _BV( PD4 )
#define SW_FORMAT     _BV( PD5 )
#define SW_MASK       ( SW_START_STOP | SW_FORMAT )
#define IR_INPUT      _BV( PD2 )

typedef enum {
    WriteToSD,
    WriteToUSART,
} WritingTarget;

static volatile uint32_t system_clock;  /* 100usごとにカウントされるタイマー */
static uint16_t input_counter;
static volatile uint8_t input;

static Devices all_sensors;
static volatile Devices enabled_dev;
static volatile Devices write_dev;
static Devices updated_dev;
static volatile WritingTarget target;

static void fatal_error( void );
static void onoff_led( void );
static void sensor_init_error( void );

ISR( USART_RX_vect )
{
    /* USART受信割り込み */
    uint8_t data = 0;

    /* 読める限り */
    while ( usart_can_read() ) {
        /* 読む */
        data = usart_read();
    }

    // Handling
    if ( data == TRANSMIT_START ) {
        if ( !write_dev ) {
            // Start data transmit
            write_dev = enabled_dev & ( DEV_MAG | DEV_GYRO | DEV_ACC | DEV_PRESS | DEV_TEMP );
            target = WriteToUSART;
        }
    } else if ( data == TRANSMIT_STOP ) {
        if ( write_dev && target == WriteToUSART ) {
            // Stop data transmit
            write_dev = 0;

            // LED off
            PORTD &= ~LED_STATUS;
        }
    } else if ( data == TRANSMIT_HANDSHAKE ) {
        // Hand shake
        while ( !usart_can_write() );
        usart_write( TRANSMIT_HANDSHAKE_ACK );
    } else {
        // Unknown date
        // fatal();
    }
}

ISR( TIMER0_COMPA_vect )
{
    /* タイマー0コンペアマッチA割り込みベクター */

    /* システムクロックを100usごとに1更新 */
    system_clock++;

    /* 50msごとに入力読み込み ( チャタリング防止 ) */
    if ( 500 <= input_counter ) {
        /* Read PD value with input pin masks */
        input = ~PIND & SW_MASK;

        input_counter = 0;
    } else {
        input_counter++;
    }
}

void fatal_error( void )
{
    /* 致命的な問題が起きたのでLED点滅 */
    while ( 1 ) {
        PORTD |= LED_STATUS;
        _delay_ms( 150 );
        PORTD &= ~LED_STATUS;
        _delay_ms( 150 );
    }
}

void sensor_init_error( void )
{
    // Not fatal, but there are failed sensor
    int i;

    for ( i = 0; i < 5; i++ ) {
        PORTD |= LED_STATUS;
        _delay_ms( 150 );
        PORTD &= ~LED_STATUS;
        _delay_ms( 150 );
    }
}

void onoff_led( void )
{
    /* LEDを少しつけて消す */
    PORTD |= LED_STATUS;
    _delay_ms( 1000 );
    PORTD &= ~LED_STATUS;
}

int main( void )
{
    /* sensor3 制御プログラム */
    int i;
    uint32_t before_system_clock;
    uint32_t now_system_clock;

    uint8_t before_input;
    uint8_t pushed_input;
    uint8_t now_input;
    uint8_t check_pushed;

    LPS25HUnit pres;
    AK8975Unit mag;
    MPU9150Unit mpu9150;

    MicomFS fs;
    MicomFSFile fp;
    char file_name[16];

    uint8_t data;
    char ret;

    /* 割り込み停止 */
    cli();

    // Global settings
    // Enable pullup
    MCUCR &= ~_BV( PUD );   // PUD is default zero

    /* ポート向き初期化 */
    DDRD = 0x80;    /* PD0-6 : 入力, PD7 : 出力 */
    DDRC = 0x00;    /* PC    : 入力 */
    DDRB = 0x00;    /* PB    : 入力 */

    /* 入力プルアップ有効 */
    PORTB = 0xFF;
    PORTC = 0xFF;
    PORTD = 0x7F;           /* PD7は出力 */

    /* タイマー0を初期化 */
    TCCR0B = 0x02;          /* プリスケール8，CTCモード */
    TCCR0A = 0x02;          /* CTCモード */
    TIMSK0 = 0x02;          /* コンペアマッチA割り込み有効 */
    OCR0A  = 100;           /* 100usごとに割りこみ発生 */

    // Initialize USART
    usart_init( 9600, UsartRX | UsartTX, UsartIntRX );

    /* I2Cバス初期化 */
    i2c_init_master( 15, I2CPrescale1, 0, 0 );

    /* デバイス初期化 */
    all_sensors = DEV_MAG | DEV_GYRO | DEV_ACC | DEV_PRESS | DEV_TEMP;
    enabled_dev = 0;

    if ( sd_init( SPIOscDiv2, 512, 0 ) ) {
        enabled_dev |= DEV_SD;
    }

    if ( mpu9150_init( &mpu9150, 0x68, 0, MPU9150LPFCFG0, MPU9150AccFSR16g, MPU9150AccHPFReset, MPU9150GyroFSR2000DPS ) ) {
        enabled_dev |= DEV_GYRO | DEV_ACC | DEV_TEMP;
    }

    if ( ak8975_init( &mag, 0x0C ) ) {
        enabled_dev |= DEV_MAG;
    }

    if ( lps25h_init( &pres, 0x5D, LPS25H25_25Hz, LPS25HPresAvg512, LPS25HTempAvg64 ) ) {
        enabled_dev |= DEV_PRESS;
    }

    /* LEDを消して少し待つ */
    PORTD &= ~LED_STATUS;
    _delay_ms( 1000 );

    /* 初期化確認 ( SDは必ず必要でセンサーはどれか一つは必要 ) */
    if ( ( enabled_dev & DEV_SD ) && ( enabled_dev & all_sensors ) ) {
        /* 初期化成功なので少し光る */
        onoff_led();
    } else {
        /* 初期化失敗なので点滅し続ける */
        fatal_error();
    }

    // If there are any failed sensor, on-off-on-off LED
    if ( ( enabled_dev & ( DEV_SD | all_sensors ) ) != ( DEV_SD | all_sensors ) ) {
        sensor_init_error();
    }

    /* ピン入力初期化 */
    input = ~PIND & SW_MASK;
    before_input = input;
    check_pushed = 0;

    /* 割り込み開始 */
    sei();

    /* 測定開始 */
    lps25h_start( &pres );
    ak8975_start( &mag );

    /* 各種変数初期化 */
    write_dev     = 0;
    before_system_clock = 0;
    now_system_clock    = 0;
    system_clock        = 0;

    /* メインループ */
    while ( 1 ) {
        /* 押されたスイッチを調べる */
        now_input = input;  // Prevent input value changing under processing
        pushed_input = ~before_input & now_input;
        before_input = now_input;

        /* スイッチがどれか押されていれば入力チェック開始 */
        if ( pushed_input ) {
            before_system_clock = now_system_clock;
            check_pushed = pushed_input;
        }

        // Buttons are pushed
        if ( now_input && check_pushed ) {
            // Handle inputs
            if ( now_input != check_pushed ) {
                // Inputs are changed before handling
                check_pushed = 0;
            } else if ( before_system_clock + 10000 < now_system_clock ) {
                /* 1秒以上押され続けたのでスイッチチェック ( 同時押しは想定していない ) */

                // Disable interrupt until buttons handling
                cli();

                if ( SW_START_STOP & now_input ) {
                    /* スタートストップボタン */
                    if ( write_dev && target == WriteToSD ) {
                        /* 書き込み中なら書き込み停止 */

                        /* 終了シグネチャ書き込み */
                        data = LOG_END_SIGNATURE;
                        micomfs_seq_fwrite( &fp, &data, 1 );

                        /* Stop file writing */
                        ret  = micomfs_stop_fwrite( &fp, 0 );
                        ret += micomfs_fclose( &fp );

                        /* 書き込み指示クリア */
                        write_dev = 0;

                        /* 確認 */
                        if ( ret ) {
                            /* 成功したので消える */
                            PORTD &= ~LED_STATUS;
                        } else {
                            /* 失敗したので停止 */
                            fatal_error();
                        }
                    } else if ( !write_dev ) {
                        /* 書き込み中でなければ書き込み開始 */

                        /* 開始するたびにFS初期化とファイル作成 */
                        ret = micomfs_init_fs( &fs );

                        /* ファイル名決定 */
                        snprintf( file_name, sizeof( file_name ), "log%d.log", (int)fs.used_entry_count );

                        /* ファイル作成 */
                        ret *= micomfs_fcreate( &fs, &fp, file_name, MICOMFS_MAX_FILE_SECOTR_COUNT );

                        /* ヘッダ作成 */
                        if ( !ret ) {
                            /* ファイルの確保失敗 */
                            fatal_error();
                        } else {
                            /* 有効デバイスリスト作成 */
                            write_dev = enabled_dev & all_sensors;

                            // Write to SD
                            target = WriteToSD;

                            /* 書き込み開始 */
                            micomfs_start_fwrite( &fp, 0 );

                            /* シグネチャ書き込み */
                            data = DEVICE_LOG_SIGNATURE;
                            micomfs_seq_fwrite( &fp, &data, 1 );

                            /* 有効デバイスリスト書き込み */
                            data = write_dev;
                            micomfs_seq_fwrite( &fp, &data, 1 );

                            /* 光る */
                            PORTD |= LED_STATUS;
                        }
                    }
                } else if ( SW_FORMAT & now_input ) {
                    /* フォーマットボタン */
                    if ( !write_dev ) {
                        /* 書き込み中でなければフォーマット開始 */
                        if ( micomfs_format( &fs, 512, sd_get_size() / sd_get_block_size(), 1024, 0 ) ) {
                            /* 成功したので少し光る */
                            onoff_led();
                        } else {
                            /* 失敗したので停止 */
                            fatal_error();
                        }
                    }
                }

                // Inputs have been handled
                check_pushed = 0;

                // Enable interrupt
                sei();
            }
        }

        /* 現在の時刻を取得 */
        cli();
        now_system_clock = system_clock;
        sei();

        /* 書き込み中でなければ測定しない */
        if ( !write_dev ) {
            continue;
        }

        /* ピコピコして動いていることを示す */
        /*
        if ( now_system_clock & 0x400 ) {
            PORTD &= ~LED_STATUS;
        } else {
            PORTD |= LED_STATUS;
        }
        */
        if ( now_system_clock & 0x1800 ) {
            PORTD &= ~LED_STATUS;
        } else {
            PORTD |= LED_STATUS;
        }

        /* センサー情報取得 */
        updated_dev = 0;

        if ( ( enabled_dev & DEV_PRESS ) && lps25h_data_ready( &pres ) ) {
            /* 気圧 */
            lps25h_read( &pres );

            updated_dev |= DEV_PRESS;
        }

        if ( ( enabled_dev & DEV_MAG ) && ak8975_data_ready( &mag ) ) {
            /* 地磁気 */
            ak8975_read( &mag );

            /* 地磁気補正 */
            ak8975_calc_adjusted_h( &mag );

            /* 測定開始 */
            ak8975_start( &mag );

            updated_dev |= DEV_MAG;
        }

        if ( ( enabled_dev & ( DEV_ACC | DEV_GYRO ) ) && mpu9150_data_ready( &mpu9150 ) ) {
            /* 加速度・温度・ジャイロ */
            mpu9150_read( &mpu9150 );

            updated_dev |= ( DEV_ACC | DEV_GYRO | DEV_TEMP );
        }

        /* 必要なら各センサーデータ処理と書き込み */
        if ( ( write_dev & DEV_PRESS ) && ( updated_dev & DEV_PRESS ) ) {
            /* 気圧書き込み */
            if ( target == WriteToSD ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_LPS331AP;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = sizeof( pres.pressure );
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &pres.pressure, sizeof( pres.pressure ) );
            } else if ( target == WriteToUSART ) {
                while ( !usart_can_write() ); usart_write( LOG_SIGNATURE );
                for ( i = 0; i < sizeof( now_system_clock ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&now_system_clock )[i] );
                }
                while ( !usart_can_write() ); usart_write( ID_LPS331AP );
                while ( !usart_can_write() ); usart_write( sizeof( pres.pressure ) );
                for ( i = 0; i < sizeof( pres.pressure ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&pres.pressure )[i] );
                }
            }
        }

        if ( ( write_dev & DEV_ACC ) && ( updated_dev & DEV_ACC ) ) {
            /* 加速度書き込み */
            if ( target == WriteToSD ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_MPU9150_ACC;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = sizeof( mpu9150.acc_x ) * 3;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &mpu9150.acc_x, sizeof( mpu9150.acc_x ) );
                micomfs_seq_fwrite( &fp, &mpu9150.acc_y, sizeof( mpu9150.acc_y ) );
                micomfs_seq_fwrite( &fp, &mpu9150.acc_z, sizeof( mpu9150.acc_z ) );
            } else if ( target == WriteToUSART ) {
                while ( !usart_can_write() ); usart_write( LOG_SIGNATURE );
                for ( i = 0; i < sizeof( now_system_clock ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&now_system_clock )[i] );
                }
                while ( !usart_can_write() ); usart_write( ID_MPU9150_ACC );
                while ( !usart_can_write() ); usart_write( sizeof( mpu9150.acc_x ) * 3 );
                for ( i = 0; i < sizeof( mpu9150.acc_x ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.acc_x )[i] );
                }
                for ( i = 0; i < sizeof( mpu9150.acc_y ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.acc_y )[i] );
                }
                for ( i = 0; i < sizeof( mpu9150.acc_z ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.acc_z )[i] );
                }
            }
        }

        if ( ( write_dev & DEV_GYRO ) && ( updated_dev & DEV_GYRO ) ) {
            /* ジャイロ書き込み */
            if ( target == WriteToSD ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_MPU9150_GYRO;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = sizeof( mpu9150.gyro_x ) * 3;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &mpu9150.gyro_x, sizeof( mpu9150.gyro_x ) );
                micomfs_seq_fwrite( &fp, &mpu9150.gyro_y, sizeof( mpu9150.gyro_y ) );
                micomfs_seq_fwrite( &fp, &mpu9150.gyro_z, sizeof( mpu9150.gyro_z ) );
            } else if ( target == WriteToUSART ) {
                while ( !usart_can_write() ); usart_write( LOG_SIGNATURE );
                for ( i = 0; i < sizeof( now_system_clock ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&now_system_clock )[i] );
                }
                while ( !usart_can_write() ); usart_write( ID_MPU9150_GYRO );
                while ( !usart_can_write() ); usart_write( sizeof( mpu9150.gyro_x ) * 3 );
                for ( i = 0; i < sizeof( mpu9150.gyro_x ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.gyro_x )[i] );
                }
                for ( i = 0; i < sizeof( mpu9150.gyro_y ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.gyro_y )[i] );
                }
                for ( i = 0; i < sizeof( mpu9150.gyro_z ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.gyro_z )[i] );
                }
            }
        }

        if ( ( write_dev & DEV_MAG ) && ( updated_dev & DEV_MAG ) ) {
            /* 地磁気書き込み */
            if ( target == WriteToSD ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_AK8975;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = sizeof( mag.adj_x ) * 3;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &mag.adj_x, sizeof( mag.adj_x ) );
                micomfs_seq_fwrite( &fp, &mag.adj_y, sizeof( mag.adj_y ) );
                micomfs_seq_fwrite( &fp, &mag.adj_z, sizeof( mag.adj_z ) );
            } else if ( target == WriteToUSART ) {
                while ( !usart_can_write() ); usart_write( LOG_SIGNATURE );
                for ( i = 0; i < sizeof( now_system_clock ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&now_system_clock )[i] );
                }
                while ( !usart_can_write() ); usart_write( ID_AK8975 );
                while ( !usart_can_write() ); usart_write( sizeof( mag.adj_x ) * 3 );
                for ( i = 0; i < sizeof( mag.adj_x ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mag.adj_x )[i] );
                }
                for ( i = 0; i < sizeof( mag.adj_y ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mag.adj_y )[i] );
                }
                for ( i = 0; i < sizeof( mag.adj_z ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mag.adj_z )[i] );
                }
            }
        }

        if ( ( write_dev & DEV_TEMP ) && ( updated_dev & DEV_TEMP ) ) {
            /* 温度書き込み */
            if ( target == WriteToSD ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_MPU9150_TEMP;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = sizeof( mpu9150.temp );
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &mpu9150.temp, sizeof( mpu9150.temp ) );
            } else if ( target == WriteToUSART ) {
                while ( !usart_can_write() ); usart_write( LOG_SIGNATURE );
                for ( i = 0; i < sizeof( now_system_clock ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&now_system_clock )[i] );
                }
                while ( !usart_can_write() ); usart_write( ID_MPU9150_TEMP );
                while ( !usart_can_write() ); usart_write( sizeof( mpu9150.temp ) );
                for ( i = 0; i < sizeof( mpu9150.temp ); i++ ) {
                    while ( !usart_can_write() ); usart_write( ( (uint8_t *)&mpu9150.temp )[i] );
                }
            }
        }
    }

    /* 終了 */
    return 0;
}
