#ifndef SHYSPECTRALDELAY_H
#define SHYSPECTRALDELAY_H

#include <cmath>
#include "shy_fft.h"
#include "fourier.h"
#include "wave.h"

#define FFT_SIZE 2048
#define LAPS      4
#define MAX_DELAY_FRAMES    30
#define SPECTRAL_RING_LEN    (MAX_DELAY_FRAMES + 1)
#define SAMPLE_RATE 48000
#define MIN_FREQ 80.f
#define MAX_FX_FREQ 2000.f

class SpectralDelay {
public:
    SpectralDelay()
        : fft(new ShyFFT<float, FFT_SIZE, RotationPhasor>())
        , window([](float phase)-> float {
              return 0.5f * (1.0f - cosf(2.0f * M_PI * phase));
          })
        , stft(new soundmath::Fourier<float, FFT_SIZE>(
              processor,
              fft,
              &window,
              LAPS,
              in,
              middle,
              out
          )) {
        fft->Init();
    }

    ~SpectralDelay() {
        delete stft;
        delete fft;
    }


    void write(float sample) {
        stft->write(sample);
    }

    float read() {
        return stft->read();
    }

private:
    // callback is called once per full STFT frame:
    // inData[0..FFT_SIZE/2-1] = real bins, inData[FFT_SIZE/2..FFT_SIZE-1] = imag bins
    static void processor(const float *inData, float *outData) {
        static float history[SPECTRAL_RING_LEN][FFT_SIZE];
        static int                 head   = 0;
        static bool                inited = false;
        static constexpr int       minBin = MIN_FREQ * FFT_SIZE / SAMPLE_RATE;
        static constexpr int       maxFxBin = MAX_FX_FREQ * FFT_SIZE / SAMPLE_RATE;
        static int                 delayDist[FFT_SIZE];

        if (!inited) {
            for (auto &d: history)
                for (float &i: d)
                    i = 0.0f;

            for (int i = 0; i < FFT_SIZE; ++i) {
                delayDist[i] = daisy::Random::GetValue() % SPECTRAL_RING_LEN;
            }

            inited = true;
        }

        std::memcpy(history[head], inData, sizeof(float) * FFT_SIZE);
        for (int k = 0; k < FFT_SIZE / 2; ++k) {
            if (k < minBin) {
                outData[k]                = 0.f; // real part
                outData[k + FFT_SIZE / 2] = 0.f; // imag part
            } else if (k < maxFxBin) {
                int readHead              = (head + delayDist[k]) % SPECTRAL_RING_LEN;
                outData[k]                = history[readHead][k];
                outData[k + FFT_SIZE / 2] = history[readHead][k + FFT_SIZE / 2];
            } else {
                outData[k]                = history[head][k] * 1.5f;
                outData[k + FFT_SIZE / 2] = history[head][k + FFT_SIZE / 2];
            }
        }
        int outIdx = (head + 1) % SPECTRAL_RING_LEN;
        head       = outIdx;
    }

    ShyFFT<float, FFT_SIZE, RotationPhasor> *fft;
    soundmath::Wave<float>                   window;
    soundmath::Fourier<float, FFT_SIZE> *    stft;
    float                                    in[FFT_SIZE * LAPS * 2];
    float                                    middle[FFT_SIZE * LAPS * 2];
    float                                    out[FFT_SIZE * LAPS * 2];
};

#endif // SHYSPECTRALDELAY_H
