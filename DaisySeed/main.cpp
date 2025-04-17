#include "daisy_seed.h"
#include "daisysp.h"
#include "include/max98389.h"
#include "include/filter3rdO.h"

using namespace daisysp;
using namespace daisy;

DaisySeed hardware;

max98389 amp;
ThirdOrderFilter * bp1;
static Oscillator osc;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    float amp_in_current;
    float amp_in_voltage;
    float oscOut = 0.0;
    for(size_t i = 0; i < size; i++)
    {
        oscOut = osc.Process();
        // amp_in_current = in[3][i];
        // amp_in_voltage = in[2][i];
        // int32_t cur_32 = f2s32(amp_in_current);
        // int16_t cur_16 = static_cast<int16_t>(cur_32 >> 16);
        // amp_in_current = s162f(cur_16);
        // float filtered = bp1->process(amp_in_current);
        out[0][i] = oscOut;
        out[1][i] = oscOut;
        out[2][i] = oscOut;
        out[3][i] = oscOut;
    }
}

int main(void)
{
    hardware.Configure();
    hardware.Init();
    hardware.StartLog(false);
    hardware.PrintLine("Starting up...\n");
    if (!amp.init())
    {
        hardware.PrintLine("ERROR: Failed to initialize MAX98389.\n");
        return -1;
    }
    System::Delay(500);
    SaiHandle external_sai_handle;
    SaiHandle::Config external_sai_cfg;
    external_sai_cfg.periph          = SaiHandle::Config::Peripheral::SAI_2;
    external_sai_cfg.sr              = SaiHandle::Config::SampleRate::SAI_48KHZ;
    external_sai_cfg.bit_depth       = SaiHandle::Config::BitDepth::SAI_32BIT;
    external_sai_cfg.a_sync          = SaiHandle::Config::Sync::SLAVE;
    external_sai_cfg.a_dir           = SaiHandle::Config::Direction::RECEIVE;
    external_sai_cfg.b_sync          = SaiHandle::Config::Sync::MASTER;
    external_sai_cfg.b_dir           = SaiHandle::Config::Direction::TRANSMIT;
    external_sai_cfg.pin_config.fs   = seed::D27;
    external_sai_cfg.pin_config.mclk  = seed::D24;
    external_sai_cfg.pin_config.sck  = seed::D28;
    external_sai_cfg.pin_config.sb   = seed::D25;
    external_sai_cfg.pin_config.sa   = seed::D26;


    /** Initialize the SAI new handle */
    if (external_sai_handle.Init(external_sai_cfg) == SaiHandle::Result::ERR || !external_sai_handle.IsInitialized())
    {
        hardware.PrintLine("ERROR: Failed to initialize SAI.\n");
        return -1;
    }

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 32;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain = 1.f;
    // audio_cfg.output_compensation = 100.f;
    if (hardware.audio_handle.Init(audio_cfg, hardware.AudioSaiHandle(), external_sai_handle) != AudioHandle::Result::OK)
    {
        hardware.PrintLine("ERROR: Failed to initialize audio handle.\n");
        return -1;
    }

    // bp1 = new ThirdOrderFilter(hardware.AudioSampleRate());
    // bp1->setFilterParams(228.8f, 10.f);
    // bp1->setFilterParams(400.8f, 10.f);

    osc.Init(hardware.AudioSampleRate());
    osc.SetWaveform(Oscillator::WAVE_SIN);
    osc.SetFreq(440.f);
    osc.SetAmp(1.f);

    hardware.StartAudio(AudioCallback);
    hardware.SetLed(true);

    while (true) { }
}
