#include "lps25h.h"

char lps25h_init( LPS25HUnit *unit, uint8_t address, LPS25HDataRate rate, LPS25HPresAvg pres_avg, LPS25HTempAvg temp_avg )
{
    /* 初期化 */
    uint8_t data;
    unit->address = address;

    /* デバイスID確認 */
    if ( !i2c_read_register( unit->address, 0x0F, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* デバイスIDが間違っていれば失敗 */
    if ( data != 0xBD ) {
        return 0;
    }

    /* マニュアルに従い一度パワーオフ */
    data = 0x00;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* 内部で平均化させる ( これを設定しないと正しく動作しない ) */
    data = pres_avg | temp_avg;

    if ( !i2c_write_register( unit->address, 0x10, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* データレート設定，読み出し中のデータロック */
    data = 0x04 | rate;

    unit->ctrl_1 = data;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_start( LPS25HUnit *unit )
{
    /* パワーダウン解除 */
    uint8_t data;

    data = unit->ctrl_1 | 0x80;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_stop( LPS25HUnit *unit )
{
    /* パワーダウン */
    uint8_t data;

    data = unit->ctrl_1 & ~0x80;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_data_ready( LPS25HUnit *unit )
{
    /* 気圧データ準備確認 */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x27, &data, 1, I2CPolling ) ) {
        return 0;
    }

    if ( data & 0x02 ) {
        return 1;
    }

    return 0;
}

char lps25h_temp_data_ready( LPS25HUnit *unit )
{
    /* 温度データ準備確認 */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x27, &data, 1, I2CPolling ) ) {
        return 0;
    }

    if ( data & 0x01 ) {
        return 1;
    }

    return 0;
}

char lps25h_read( LPS25HUnit *unit )
{
    /* 気圧データーを読む */
    uint8_t data[4];
    data[3] = 0;

    /* マルチバイトリードを行うにはMSBを1にする必要がある */
    if ( !i2c_read_register( unit->address, 0x28 | 0x80, data, 3, I2CPolling ) ) {
        return 0;
    }

    /* 本来int32だが、strict-aliasing rulesに抵触するため環境定義動作に依存するがこう書く */
    unit->pressure = *(uint32_t *)( data );

    return 1;
}

char lps25h_read_temp( LPS25HUnit *unit )
{
    /* 温度データーを読む */
    uint8_t data[2];

    /* マルチバイトリードを行うにはMSBを1にする必要がある */
    if ( !i2c_read_register( unit->address, 0x2B | 0x80, data, 2, I2CPolling ) ) {
        return 0;
    }

    unit->temp = *(uint16_t *)( data );

    return 1;
}


char lps25h_one_shot( LPS25HUnit *unit )
{
    /* Start one shot acquisition */
    uint8_t data;

    data = 0x01;

    if ( !i2c_write_register( unit->address, 0x21, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}
