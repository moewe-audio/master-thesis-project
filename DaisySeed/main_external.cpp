#include "daisy_seed.h"
#include "daisysp.h"
#include "include/max98389.h"
#include "include/RMSController.h"
#include "lib/DaisySP/Source/Utility/dsp.h"
#include "include/filterUtility.h"

#define PRINT_DECIMATION 1024 // Print one value every 512 samples
#define BOOT_RAMP_UP_SECS 4

using namespace daisysp;
using namespace daisy;

DaisySeed hardware;

max98389                    amp;
float                       bootRampUpGain      = 0.f;
float                       bootRampUpIncrement = 0.f;
int                         bootRampUpSamples;
int                         bootRampUpCounter = 0;
static RmsTrackerController rmsCtrl(1.f, 20, 0.001, 0, 0.f, 1.f);
static BiQuad               highpass1(48000);
static BiQuad               highpass2(48000);
static BiQuad               lowpass(48000);

void ledErrorPulse(int n) {
    for (int i = 0; i < n; i++) {
        hardware.SetLed(true);
        System::Delay(100);
        hardware.SetLed(false);
        System::Delay(100);
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    float ampOut   = 0.0;
    bootRampUpGain = fclamp(bootRampUpGain + bootRampUpIncrement, 0.f, 1.f);
    for (size_t i = 0; i < size; i++) {
        float piezoIn = in[0][i];
        piezoIn       = lowpass.process(highpass2.process(highpass1.process(piezoIn)));
        float extIn   = out[1][i];
        // float gain    = rmsCtrl.processSample(extIn);
        // ampOut        = bootRampUpGain * extIn * gain;
        out[0][i] = piezoIn;
        out[1][i] = extIn;
        out[2][i] = extIn;
    }
}


int main(void) {
    hardware.Configure();
    hardware.Init();
    hardware.StartLog(false);

    System::Delay(1000);

    auto err = amp.init(hardware, false);
    if (err != max98389::ERROR_CODE::NO_ERROR) {
        hardware.PrintLine("ERROR: Failed to initialize MAX98389.\n");
        ledErrorPulse(err);
        return -1;
    }

    SaiHandle         external_sai_handle;
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

    if (!external_sai_handle.IsInitialized()) {
        ledErrorPulse(3);
        return -1;
    }

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 128;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 1.f;
    hardware.audio_handle.Init(audio_cfg, hardware.AudioSaiHandle(), external_sai_handle);

    bootRampUpSamples   = BOOT_RAMP_UP_SECS * hardware.AudioSampleRate();
    bootRampUpIncrement = 1.f / (float) bootRampUpSamples * hardware.AudioBlockSize();

    highpass1.setCoefficients(1.984079851482528,
                              0.9842055778154112,
                              0.9920713573244848,
                              1.9841427146489696,
                              0.9920713573244848
    ); // fc=86 Q=0.707
    highpass2.setCoefficients(1.984079851482528,
                              0.9842055778154112,
                              0.9920713573244848,
                              1.9841427146489696,
                              0.9920713573244848
    ); // fc=86 Q=0.707
    lowpass.setCoefficients(
        -1.5431183905443995,
        0.6296011104506473,
        0.021620679976561967,
        0.043241359953123934,
        0.021620679976561967
    ); // fc=2500 Q=0.707

    hardware.StartAudio(AudioCallback);
    hardware.SetLed(true);

    while (true) {
    }
}
