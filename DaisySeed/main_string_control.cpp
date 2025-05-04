#include "daisy_seed.h"
#include "daisysp.h"
#include "include/max98389.h"
#include "include/filterUtility.h"
#include "include/RunningRms.h"
#include "lib/DaisySp/Source/Filters/onepole.h"
#include "include/RMSController.h"
#include "include/DynamicRangeLimiter.h"

#define PRINT_DECIMATION 1024 // Print one value every 512 samples
#define BOOT_RAMP_UP_SECS 4

using namespace daisysp;
using namespace daisy;

DaisySeed hardware;

max98389 amp;
FourthOrderFilter *deRumble1;
FourthOrderFilter *deRumble2;
FourthOrderFilter *eStringdamper;
FourthOrderFilter *body_filter;
FourthOrderFilter *shelf_hp_250;
FourthOrderFilter *notch_220;
FourthOrderFilter *lp_12000;
float bootRampUpGain = 0.f;
float bootRampUpIncrement = 0.f;
int bootRampUpSamples;
int bootRampUpCounter = 0;
static DelayLine<float, 55000> delay;
static RmsTrackerController rms_tracker(1.f, 400, 0.02, 0, 0, 1, 48000);

void ledErrorPulse(int n) {
    for (int i = 0; i < n; i++) {
        hardware.SetLed(true);
        System::Delay(100);
        hardware.SetLed(false);
        System::Delay(100);
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    float ampOut = 0.0;
    bootRampUpGain = fclamp(bootRampUpGain + bootRampUpIncrement, 0.f, 1.f);
    for (size_t i = 0; i < size; i++) {
        float piezo_in = in[0][i];
        float current_in = in[2][i];
        piezo_in = deRumble1->process(piezo_in);
        piezo_in = deRumble2->process(piezo_in);
        // piezo_in = body_filter->process(piezo_in);
        // piezo_in = shelf_hp_250->process(piezo_in);
        // piezo_in = notch_220->process(piezo_in);
        // piezo_in = lp_12000->process(piezo_in);

        // piezo_in -= voltage_in;
        // float gain = rms_tracker.processSample(piezo_in);

        // delay.Write(piezo_in);
        // ampOut = delay.Read();
        // ampOut = gain * piezo_in;
        // ampOut += current_in * 0.1f;
        ampOut *= bootRampUpGain;
        out[0][i] = piezo_in * 2.f;
        out[1][i] = current_in;
        out[2][i] = ampOut;
        // if (++sample_counter >= PRINT_DECIMATION)
        // {
        //     sample_counter = 0;
        //     hardware.PrintLine(">RMS:" FLT_FMT(8), FLT_VAR(8,bp1_gain));
        // }
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

    SaiHandle external_sai_handle;
    SaiHandle::Config external_sai_cfg{};
    external_sai_cfg.periph = SaiHandle::Config::Peripheral::SAI_2;
    external_sai_cfg.sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
    external_sai_cfg.bit_depth = SaiHandle::Config::BitDepth::SAI_16BIT;
    external_sai_cfg.a_sync = SaiHandle::Config::Sync::SLAVE;
    external_sai_cfg.a_dir = SaiHandle::Config::Direction::RECEIVE;
    external_sai_cfg.b_sync = SaiHandle::Config::Sync::MASTER;
    external_sai_cfg.b_dir = SaiHandle::Config::Direction::TRANSMIT;
    external_sai_cfg.pin_config.fs = seed::D27;
    external_sai_cfg.pin_config.mclk = seed::D24;
    external_sai_cfg.pin_config.sck = seed::D28;
    external_sai_cfg.pin_config.sb = seed::D25;
    external_sai_cfg.pin_config.sa = seed::D26;

    /** Initialize the SAI new handle */
    external_sai_handle.Init(external_sai_cfg);

    if (!external_sai_handle.IsInitialized()) {
        ledErrorPulse(3);
        return -1;
    }

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize = 4;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain = 1.f;
    hardware.audio_handle.Init(audio_cfg, hardware.AudioSaiHandle(), external_sai_handle);

    deRumble1 = new FourthOrderFilter(hardware.AudioSampleRate());
    deRumble2 = new FourthOrderFilter(hardware.AudioSampleRate());
    deRumble1->setCoefficients(deRumble1->rumbleFilter1[0], deRumble1->rumbleFilter1[1], deRumble1->rumbleFilter1[2],
                               deRumble1->rumbleFilter1[3], deRumble1->rumbleFilter1[4]);
    deRumble2->setCoefficients(deRumble2->rumbleFilter2[0], deRumble2->rumbleFilter2[1], deRumble2->rumbleFilter2[2],
                               deRumble2->rumbleFilter2[3], deRumble2->rumbleFilter2[4]);

    eStringdamper = new FourthOrderFilter(hardware.AudioSampleRate());
    eStringdamper->setFilterParams(82.5f, 20);

    body_filter = new FourthOrderFilter(hardware.AudioSampleRate());
    body_filter->setCoefficients(-1.9982182418079835, 0.9984566369359177, 0.9992283184679588, -1.9982182418079835,
                                 0.9992283184679588);

    shelf_hp_250 = new FourthOrderFilter(hardware.AudioSampleRate());
    shelf_hp_250->setCoefficients(-1.9177607157299796
                                  , 0.9210113426950024, 0.9821592923440207
                                  , -1.9188720599608393, 0.937740706120122
    );


    notch_220 = new FourthOrderFilter(hardware.AudioSampleRate());
    notch_220->setCoefficients(-1.9962966728219715, 0.9971247442620121, 0.9985623721310061, -1.9962966728219715,
                               0.9985623721310061
    );

    lp_12000 = new FourthOrderFilter(hardware.AudioSampleRate());
    lp_12000->setCoefficients(-1.300625813438253e-16, 0.17149958574979282, 0.2928748964374482, 0.5857497928748964,
                              0.5857497928748964

    );


    bootRampUpSamples = BOOT_RAMP_UP_SECS * hardware.AudioSampleRate();
    bootRampUpIncrement = 1.f / (float) bootRampUpSamples * hardware.AudioBlockSize();

    delay.Init();
    delay.SetDelay((size_t) 55000);

    hardware.StartAudio(AudioCallback);
    hardware.SetLed(true);

    while (true) {
    }
}
