#ifndef SIGMUND_H
#define SIGMUND_H

#include "arm_const_structs.h" // for arm_cfft_sR_f32_len1024
#include "arm_math.h"
#include <cmath> // for std::ceil, std::floor
#include <vector>

#define FFT_SIZE 4096

struct SpectralPeak {
    float freq;
    float amp;
};

struct Track {
    int   id;
    float freq;
    float amp;
    int   missedFrames;
};

class Sigmund {
public:
    static constexpr int   kHopSize        = 1024;
    static constexpr int   nPeaks          = 14;
    static constexpr int   nTracks         = 14;
    static constexpr float minFreqHz       = 85.0f;
    static constexpr float maxFreqHz       = 7500.0f;
    static constexpr float gainThresh      = 0.01f;
    static constexpr int   maxMissedFrames = 3;
    static constexpr float freqToleranceHz = 100.f;

    Sigmund() { init(48000.0f); }

    ~Sigmund() = default;

    void init(float sampleRate) {
        sampleRate_  = sampleRate;
        bufferIndex_ = 0;
        nextTrackId_ = 0;
        tracks_.clear();
        // Precompute Hann window
        for (int i = 0; i < FFT_SIZE; ++i) {
            hannWindow[i] =
                    0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
        }
    }

    bool processSample(float in) {
        ringBuffer_[bufferIndex_] = in;
        bufferIndex_              = (bufferIndex_ + 1) & (FFT_SIZE - 1);
        if ((bufferIndex_ & (kHopSize - 1)) == 0) {
            runFrame();
            return true;
        }
        return false;
    }

    const std::vector<Track> &getTracks() const { return tracks_; }

private:
    void runFrame() {
        // Window & interleave into complex buffer
        int idx = bufferIndex_;
        for (int i = 0; i < FFT_SIZE; ++i) {
            int j              = (idx + i) & (FFT_SIZE - 1);
            fftBuf_[2 * i]     = ringBuffer_[j] * hannWindow[i]; // real
            fftBuf_[2 * i + 1] = 0.0f; // imag
        }

        // Complex FFT (in-place)
        arm_cfft_f32(&arm_cfft_sR_f32_len2048, fftBuf_, 0, 1);

        // Magnitude of all FFT_SIZE complex bins
        arm_cmplx_mag_f32(fftBuf_, mag_, FFT_SIZE);

        // Peak detection over the half-spectrum
        constexpr int half = FFT_SIZE >> 1;
        SpectralPeak  detectedBuf[nPeaks + 1];
        int           detCount = 0;
        int           startBin =
                std::max(1, (int) std::ceil(minFreqHz * FFT_SIZE / sampleRate_));
        int hz180Bin =
                std::max(1, (int) std::ceil(180 * FFT_SIZE / sampleRate_));
        int hz200Start =
                std::max(1, (int) std::ceil(180 * FFT_SIZE / sampleRate_));
        int hz200End =
                std::max(1, (int) std::ceil(210 * FFT_SIZE / sampleRate_));
        int hz230Start =
                std::max(1, (int) std::ceil(220 * FFT_SIZE / sampleRate_));
        int hz230End =
                std::max(1, (int) std::ceil(240 * FFT_SIZE / sampleRate_));
        int hz560Start =
                std::max(1, (int) std::ceil(550 * FFT_SIZE / sampleRate_));
        int hz560End =
                std::max(1, (int) std::ceil(570 * FFT_SIZE / sampleRate_));
        int endBin =
                std::min(half - 1, (int) std::floor(maxFreqHz * FFT_SIZE / sampleRate_));

        for (int i = startBin; i <= endBin; ++i) {
            float freq = i * (sampleRate_ / FFT_SIZE);
            float m0   = mag_[i - 1];
            float m1   = mag_[i];
            float m2   = mag_[i + 1];
            // float wM0   = semitoneWeight((i - 1) * (sampleRate_ / FFT_SIZE));
            // float wM1   = semitoneWeight(i * (sampleRate_ / FFT_SIZE));
            // float wM2   = semitoneWeight((i + 1) * (sampleRate_ / FFT_SIZE));

            if (m1 > gainThresh && m1 > m0 && m1 > m2) {
                float denom     = (m0 - 2 * m1 + m2);
                float delta     = (denom == 0.0f) ? 0.0f : 0.5f * (m0 - m2) / denom;
                float interpMag = m1 - 0.25f * (m0 - m2) * delta;
                // insert sorted by amplitude
                int slot = detCount;
                if (detCount < nPeaks)
                    ++detCount;
                while (slot > 0 && interpMag > detectedBuf[slot - 1].amp) {
                    detectedBuf[slot] = detectedBuf[slot - 1];
                    --slot;
                }
                detectedBuf[slot] = {freq, interpMag};
            }
        }

        for (int i = 0; i < nPeaks; ++i) {
            fftPeaks_[i] = (i < detCount) ? detectedBuf[i] : SpectralPeak{0.0f, 0.0f};
        }
        lastDetectedCount = detCount;

        updateTracks();
    }

    void updateTracks() {
        std::vector<bool> updated(tracks_.size(), false);
        std::vector<bool> used(lastDetectedCount, false);

        // match existing
        for (int pi = 0; pi < lastDetectedCount; ++pi) {
            float pf     = fftPeaks_[pi].freq;
            int   bestTi = -1;
            float bestD  = freqToleranceHz;
            for (size_t ti = 0; ti < tracks_.size(); ++ti) {
                if (updated[ti])
                    continue;
                float d = std::fabs(tracks_[ti].freq - pf);
                if (d < bestD) {
                    bestD  = d;
                    bestTi = ti;
                }
            }
            if (bestTi >= 0) {
                auto &tr        = tracks_[bestTi];
                tr.freq         = pf;
                tr.amp          = fftPeaks_[pi].amp;
                tr.missedFrames = 0;
                updated[bestTi] = true;
                used[pi]        = true;
            }
        }

        // spawn new
        for (int pi = 0; pi < lastDetectedCount; ++pi) {
            if (!used[pi] && tracks_.size() < nTracks) {
                tracks_.push_back(
                    {nextTrackId_++, fftPeaks_[pi].freq, fftPeaks_[pi].amp, 0});
            }
        }

        // cull old
        for (int i = (int) tracks_.size() - 1; i >= 0; --i) {
            if (!updated[i] && ++tracks_[i].missedFrames > maxMissedFrames) {
                tracks_.erase(tracks_.begin() + i);
            }
        }
    }

    static float semitoneWeight(float freq) {
        if (freq < 20.0f || freq > 5000.0f) return 0.0f;
        float midiNote    = 69.0f + 12.0f * log2f(freq / 440.0f);
        float nearestNote = roundf(midiNote);
        float nearestFreq = 440.0f * powf(2.0f, (nearestNote - 69.0f) / 12.0f);
        float centsDiff   = 1200.0f * log2f(freq / nearestFreq);
        float sigmaCents  = 80.0f;
        float weight      = expf(-0.5f * (centsDiff * centsDiff) / (sigmaCents * sigmaCents));
        return weight;
    }

    float              sampleRate_;
    float32_t          fftBuf_[FFT_SIZE * 2];
    float              mag_[FFT_SIZE];
    SpectralPeak       fftPeaks_[nPeaks];
    int                lastDetectedCount = 0;
    int                bufferIndex_      = 0;
    float              ringBuffer_[FFT_SIZE];
    float              hannWindow[FFT_SIZE];
    int                nextTrackId_ = 0;
    std::vector<Track> tracks_;
};

#endif // SIGMUND_H
