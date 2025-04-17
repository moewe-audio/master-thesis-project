#include "daisy_seed.h"

using namespace daisy;

class max98389
{
public:
    const uint8_t slave_address =               0x38 << 1;
    const uint16_t revision_id_register =       0x22FF;
    const uint8_t expected_revision_id =        0x41;
    const uint16_t pcm_mode_register =          0x2040;
    const uint16_t pcm_clock_register =         0x2041;
    const uint16_t pcm_sample_rate_register =   0x2042;
    const uint16_t pcm_vmon_slots_register =    0x2044;
    const uint16_t pcm_imon_slots_register =    0x2045;
    const uint16_t tx_hi_z_control1 =           0x2050;
    const uint16_t pcm_tx_source_en_register =  0x205D;
    const uint16_t pcm_rx_en_register =         0x205E;
    const uint16_t pcm_tx_en_register =         0x205F;
    const uint16_t speaker_channel_cfg =     0x2091;
    const uint16_t speaker_amp_output_cfg =     0x2092;
    const uint16_t speaker_dig_vol_output_cfg =     0x2090;
    const uint16_t amp_en_register =            0x209F;
    const uint16_t iv_data_dsp_control =        0x20A0;
    const uint16_t iv_data_en_register =        0x20A7;
    const uint16_t auto_recovery_register =     0x210E;
    const uint16_t global_en_register =         0x210F;

    I2CHandle device;
    const int TIMEOUT = 500;

    bool init();
};