#include "lps25h.h"

char lps25h_init( LPS25HUnit *unit, uint8_t address, LPS25HDataRate rate, LPS25HPresAvg pres_avg, LPS25HTempAvg temp_avg )
{
    /* ������ */
    uint8_t data;
    unit->address = address;

    /* �f�o�C�XID�m�F */
    if ( !i2c_read_register( unit->address, 0x0F, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* �f�o�C�XID���Ԉ���Ă���Ύ��s */
    if ( data != 0xBD ) {
        return 0;
    }

    /* �}�j���A���ɏ]����x�p���[�I�t */
    data = 0x00;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* �����ŕ��ω������� ( �����ݒ肵�Ȃ��Ɛ��������삵�Ȃ� ) */
    data = pres_avg | temp_avg;

    if ( !i2c_write_register( unit->address, 0x10, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* �f�[�^���[�g�ݒ�C�ǂݏo�����̃f�[�^���b�N */
    data = 0x04 | rate;

    unit->ctrl_1 = data;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_start( LPS25HUnit *unit )
{
    /* �p���[�_�E������ */
    uint8_t data;

    data = unit->ctrl_1 | 0x80;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_stop( LPS25HUnit *unit )
{
    /* �p���[�_�E�� */
    uint8_t data;

    data = unit->ctrl_1 & ~0x80;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char lps25h_data_ready( LPS25HUnit *unit )
{
    /* �C���f�[�^�����m�F */
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
    /* ���x�f�[�^�����m�F */
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
    /* �C���f�[�^�[��ǂ� */
    uint8_t data[4];
    data[3] = 0;

    /* �}���`�o�C�g���[�h���s���ɂ�MSB��1�ɂ���K�v������ */
    if ( !i2c_read_register( unit->address, 0x28 | 0x80, data, 3, I2CPolling ) ) {
        return 0;
    }

    /* �{��int32�����Astrict-aliasing rules�ɒ�G���邽�ߊ���`����Ɉˑ����邪�������� */
    unit->pressure = *(uint32_t *)( data );

    return 1;
}

char lps25h_read_temp( LPS25HUnit *unit )
{
    /* ���x�f�[�^�[��ǂ� */
    uint8_t data[2];

    /* �}���`�o�C�g���[�h���s���ɂ�MSB��1�ɂ���K�v������ */
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
