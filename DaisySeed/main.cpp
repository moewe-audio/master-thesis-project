#include "daisy_seed.h"
#include "daisysp.h"
#include "include/max98389.h"
#include "include/filter3rdO.h"

#define PRINT_DECIMATION 1024 // Print one value every 512 samples

using namespace daisysp;
using namespace daisy;

DaisySeed hardware;

max98389 amp;
ThirdOrderFilter * bp1;
static Oscillator osc;
static size_t sample_counter = 0;
static daisysp::DelayLine<float, 48000> delay_line;

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
    float velocity;
    float ampOut = 0.0;
    for(size_t i = 0; i < size; i++)
    {
        velocity = in[2][i]; // Assume current readings to be proportional to velocity
        // delay_line.Write(velocity);
        // ampOut = delay_line.Read() * 1.f;
        ampOut = bp1->process(velocity) * 1.f;
        out[0][i] = velocity;
        out[1][i] = velocity;
        out[2][i] = ampOut;
        out[3][i] = ampOut;
        // if (++sample_counter >= PRINT_DECIMATION)
        // {
        //     sample_counter = 0;
        //     hardware.PrintLine(">velocity:" FLT_FMT(8), FLT_VAR(8,velocity));
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

    hardware.StartAudio(AudioCallback);

    osc.Init(hardware.AudioSampleRate());
    osc.SetWaveform(osc.WAVE_SIN);
    osc.SetFreq(220);
    osc.SetAmp(0.02f);

    delay_line.Init();
    delay_line.SetDelay((size_t)48000);

    bp1->setFilterParams(225.f, 10.f);

    hardware.SetLed(true);
    while (true) { }
}