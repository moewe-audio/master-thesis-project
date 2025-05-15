#ifndef SHYSPECTRALDELAY_H
#define SHYSPECTRALDELAY_H

#include <cmath>
#include "shy_fft.h"
#include "fourier.h"
#include "wave.h"
#include "binTracking.h"
#include "Constants.h"

#define MIN_FREQ 120.f
#define MIN_FX_FREQ 121.f
#define MAX_FX_FREQ 7500.f
#define MAX_DELAY_FRAMES    70
#define SPECTRAL_RING_LEN    (MAX_DELAY_FRAMES + 1)


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
        static int           head     = 0;
        static bool          inited   = false;
        static constexpr int minBin   = MIN_FREQ * FFT_SIZE / SAMPLE_RATE;
        static constexpr int minFxBin = MIN_FX_FREQ * FFT_SIZE / SAMPLE_RATE;
        static constexpr int maxFxBin = MAX_FX_FREQ * FFT_SIZE / SAMPLE_RATE;
        static constexpr int noFxBins = (maxFxBin - minFxBin + 1);
        static float         history[SPECTRAL_RING_LEN][noFxBins * 2];
        static int           delayDist[FFT_SIZE];
        static BinTracking   magCtrl;
        static float magGains[FFT_SIZE/2];

        if (!inited) {
            magCtrl.init(MIN_FX_FREQ, MAX_FX_FREQ);
            for (auto &d: history)
                for (float &i: d)
                    i = 0.0f;

            for (int i = 0; i < FFT_SIZE / 2; ++i) {
                delayDist[i] = daisy::Random::GetValue() % SPECTRAL_RING_LEN;
            }

            inited = true;
        }
        magCtrl.process(inData, magGains);
        for (int k = 0; k < FFT_SIZE / 2; ++k) {
            if (k <= minBin) {
                outData[k]                = inData[k] * 0;
                outData[k + FFT_SIZE / 2] = inData[k + FFT_SIZE / 2] * 0;
            } else if (k >= minFxBin && k <= maxFxBin) {
                int offsetK                    = k - minFxBin;
                history[head][2 * offsetK]     = inData[k]; // real
                history[head][2 * offsetK + 1] = inData[k + FFT_SIZE / 2]; // imag

                int readHead              = (head + delayDist[k]) % SPECTRAL_RING_LEN;
                outData[k]                = magGains[k] * history[readHead][2 * offsetK]; // real
                outData[k + FFT_SIZE / 2] = history[readHead][2 * offsetK + 1]; // imag
            } else {
                outData[k]                = inData[k];
                outData[k + FFT_SIZE / 2] = inData[k + FFT_SIZE / 2];
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
