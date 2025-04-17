#include "daisy_seed.h"
#include "include/max98389.h"

using namespace daisy;

bool max98389::init()
{
    I2CHandle::Config i2c_conf;
    i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.speed  = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.mode   = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.pin_config.scl  = {DSY_GPIOB, 8};
    i2c_conf.pin_config.sda  = {DSY_GPIOB, 9};
    if (device.Init(i2c_conf) != I2CHandle::Result::OK)
    {
        return false;
    }
    uint8_t revisionId;
    auto result = device.ReadDataAtAddress(slave_address, revision_id_register, 2, &revisionId, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK || revisionId != expected_revision_id)
    {
        return false;
    }
    uint8_t data = 0b11000000; // 32 bit
    result = device.WriteDataAtAddress(slave_address, pcm_mode_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0b00000100;
    result = device.WriteDataAtAddress(slave_address, pcm_clock_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0b10001000; // 48 khZ I/V ; 48kHz audio
    result = device.WriteDataAtAddress(slave_address, pcm_sample_rate_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x00;
    result = device.WriteDataAtAddress(slave_address, pcm_vmon_slots_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x01;
    result = device.WriteDataAtAddress(slave_address, pcm_imon_slots_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x03;
    result = device.WriteDataAtAddress(slave_address, pcm_tx_source_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x01;
    result = device.WriteDataAtAddress(slave_address, pcm_rx_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x03;
    result = device.WriteDataAtAddress(slave_address, pcm_tx_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x01;
    result = device.WriteDataAtAddress(slave_address, auto_recovery_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x01;
    result = device.WriteDataAtAddress(slave_address, amp_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x03;
    result = device.WriteDataAtAddress(slave_address, iv_data_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }
    data = 0x00;
    for(uint16_t i = 0; i < 7; i++){
        device.WriteDataAtAddress(slave_address, (uint16_t)(tx_hi_z_control1+i), 2, &data, 1, TIMEOUT);
    }
    data = 0x01;
    result = device.WriteDataAtAddress(slave_address, global_en_register, 2, &data, 1, TIMEOUT);
    if (result != I2CHandle::Result::OK)
    {
        return false;
    }

    return true;
}

