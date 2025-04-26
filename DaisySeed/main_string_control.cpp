#include "daisy_seed.h"
#include "daisysp.h"
#include "include/max98389.h"
#include "include/filterUtility.h"
#include "include/RunningRms.h"
#include "lib/DaisySp/Source/Filters/onepole.h"
#include "lib/libDaisy/src/hid/logger.h"

#define PRINT_DECIMATION 1024 // Print one value every 512 samples
#define BOOT_RAMP_UP_SECS 4

using namespace daisysp;
using namespace daisy;

DaisySeed hardware;

max98389 amp;
ThirdOrderFilter * bp1;
ThirdOrderFilter * bp2;
ThirdOrderFilter * bp3;
static size_t sample_counter = 0;
static OnePole bLp, bHp, rmsLp;
static RunningRMS<128> rms;
float bootRampUpGain = 0.f;
float bootRampUpIncrement = 0.f;
int bootRampUpSamples;
int bootRampUpCounter = 0;
float targetRms = 0.6;
float bp1_gain = 0.f;
float bp1_gain_increment = 0.00001f;

void ledErrorPulse(int n)
{
    for (int i = 0; i < n; i++)
    {
        hardware.SetLed(true);
        System::Delay(100);
        hardware.SetLed(false);
        System::Delay(100);
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    float ampOut = 0.0;
    bootRampUpGain = fclamp(bootRampUpGain + bootRampUpIncrement, 0.f, 1.f);
    for(size_t i = 0; i < size; i++)
    {
        float in_current = in[2][i]; // Assume current readings to be proportional to velocity
        in_current = bLp.Process(in_current);
        in_current = bHp.Process(in_current);
        float x_bp = bp1->process(in_current);
        rms.addSample(x_bp);
        float rmsVal = rmsLp.Process(rms.getRMS());
        float error = targetRms - rmsVal;
        bp1_gain += (error > 0.f) ? bp1_gain_increment : -bp1_gain_increment;
        ampOut = bp1_gain * bp1->process(in_current);
        ampOut *= bootRampUpGain;
        out[0][i] = in_current * 10.f;
        out[1][i] = ampOut;
        out[2][i] = ampOut;
        out[3][i] = ampOut;
        // if (++sample_counter >= PRINT_DECIMATION)
        // {
        //     sample_counter = 0;
        //     hardware.PrintLine(">RMS:" FLT_FMT(8), FLT_VAR(8,bp1_gain));
        // }
    }
}


int main(void)
{
    hardware.Configure();
    hardware.Init();
    hardware.StartLog(false);

    System::Delay(1000);

    auto err = amp.init(hardware, false);
    if (err != max98389::ERROR_CODE::NO_ERROR)
    {
        hardware.PrintLine("ERROR: Failed to initialize MAX98389.\n");
        ledErrorPulse(err);
        return -1;
    }

    SaiHandle external_sai_handle;
    SaiHandle::Config external_sai_cfg;
    external_sai_cfg.periph          = SaiHandle::Config::Peripheral::SAI_2;
    external_sai_cfg.sr              = SaiHandle::Config::SampleRate::SAI_48KHZ;
    external_sai_cfg.bit_depth       = SaiHandle::Config::BitDepth::SAI_16BIT;
    external_sai_cfg.a_sync          = SaiHandle::Config::Sync::SLAVE;
    external_sai_cfg.a_dir           = SaiHandle::Config::Direction::RECEIVE;
    external_sai_cfg.b_sync          = SaiHandle::Config::Sync::MASTER;
    external_sai_cfg.b_dir           = SaiHandle::Config::Direction::TRANSMIT;
    external_sai_cfg.pin_config.fs   = seed::D27;
    external_sai_cfg.pin_config.mclk = seed::D24;
    external_sai_cfg.pin_config.sck  = seed::D28;
    external_sai_cfg.pin_config.sb   = seed::D25;
    external_sai_cfg.pin_config.sa   = seed::D26;

    /** Initialize the SAI new handle */
    external_sai_handle.Init(external_sai_cfg);

    if (!external_sai_handle.IsInitialized())
    {
        ledErrorPulse(3);
        return -1;
    }

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 4;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 1.f;
    hardware.audio_handle.Init(audio_cfg, hardware.AudioSaiHandle(), external_sai_handle);

    bp1 = new ThirdOrderFilter(hardware.AudioSampleRate());
    bp2 = new ThirdOrderFilter(hardware.AudioSampleRate());
    bp3 = new ThirdOrderFilter(hardware.AudioSampleRate());
    bp1->setFilterParams(83.f, 5.f);
    bp2->setFilterParams(83.f * 2.f, 20.f);
    bp3->setFilterParams(565.f, 20.f);

    bLp.Init();
    bLp.SetFilterMode(OnePole::FILTER_MODE_LOW_PASS);
    bLp.SetFrequency(10000.f / hardware.AudioSampleRate()); // onepole impl requires ratio
    bHp.Init();
    bHp.SetFilterMode(OnePole::FILTER_MODE_HIGH_PASS);
    bHp.SetFrequency(40.f / hardware.AudioSampleRate()); // see above
    rmsLp.Init();
    rmsLp.SetFilterMode(OnePole::FILTER_MODE_LOW_PASS);
    rmsLp.SetFrequency(500.f / hardware.AudioSampleRate()); // see above

    bootRampUpSamples = BOOT_RAMP_UP_SECS * hardware.AudioSampleRate();
    bootRampUpIncrement = 1.f / (float) bootRampUpSamples * hardware.AudioBlockSize();

    hardware.StartAudio(AudioCallback);
    hardware.SetLed(true);

    while (true) { }
}
