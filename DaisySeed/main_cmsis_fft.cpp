#include "daisy_seed.h"
#include "lib/libDaisy/src/daisy_seed.h"
#include "lib/libDaisy/src/dev/sdram.h"
#include "include/cmsisFFT.h"

using namespace daisy;

// #define FFT_SIZE    2048
FFTProcessor fft;
// arm_rfft_fast_instance_f32  fftInst;      // the FFT “plan”
// scratch buffer for real-only RFFT of length FFT_SIZE
// float32_t                   buf[FFT_SIZE];

// if you also want the precomputed tables in SDRAM, see below…

DaisySeed hw;

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    // assume stereo interleaved, mono processing
    // copy left channel into buf[]
    for(size_t i = 0; i*2 < size; i++) {
        out[i*2] = fft.processSample(in[i*2]);
        out[i*2+1] = in[i*2];
    }


    // forward  real→complex FFT
    // arm_rfft_fast_f32(&fftInst, buf, buf, 0);

    // … (you could mess with magnitude/phase here!) …

    // inverse complex→real IFFT
    // arm_rfft_fast_f32(&fftInst, buf, buf, 1);

    // write back to both channels
    // for(size_t i = 0; i < FFT_SIZE && i*2 < size; i++)
    // {
    //     float32_t y = buf[i];
    //     out[i*2    ] = y;
    // }
}

int main(void)
{
    // 1) bring up everything
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(256);
    // 3) init the FFT “instance” for our block size
    // arm_rfft_fast_init_f32(&fftInst, FFT_SIZE);
    fft.init();
    // 4) start audio and spin
    hw.StartAudio(AudioCallback);
    hw.SetLed(true);
    for(;;) { /* nothing else to do */ }
}
