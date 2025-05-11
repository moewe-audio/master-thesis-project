#ifndef MARTHA_H
#define MARTHA_H

#pragma once

#include <ratio>
#include <vector>
#include "daisysp.h"
#include "filterUtility.h"
#include "Sigmund.h"

using namespace daisysp;

struct Voice {
    bool    active  = false;
    int     trackId = -1;
    BiQuad *bpf;
    Adsr    env;
    float   amp  = 0.0f;
    bool    gate = false;

    void init(float sample_rate) {
        bpf = new BiQuad(sample_rate);
        env.Init(sample_rate);
        env.SetTime(ADSR_SEG_ATTACK, 0.005f);
        env.SetTime(ADSR_SEG_DECAY, 0.100f);
        env.SetSustainLevel(0.6f);
        env.SetTime(ADSR_SEG_RELEASE, 0.200f);
    }

    void noteOn(int id, float freq, float velocity) {
        trackId = id;
        amp     = velocity;
        bpf->setFilterParams(freq, 20.f);
        active = true;
        gate   = true;
    }

    void noteOff(int id) {
        if (active && trackId == id)
            gate = false;
    }

    float process(float input) {
        if (!active)
            return 0.0f;

        // float out = bpf->process(input) * env.Process(gate) * amp;
        env.Process(gate);
        float out = bpf->process(input);
        if (!env.IsRunning())
            active = false;
        return out;
    }
};

class Martha {
public:
    explicit Martha(int maxVoices = 32)
        : maxVoices_(maxVoices)
        , ampThresh_(0.01f) {
        // trackFlagsPrev_.fill(-1);
        // trackSwitches_.fill(1);
    }

    void init(float sampleRate) {
        voices_.resize(maxVoices_);
        for (auto &v: voices_)
            v.init(sampleRate);
    }

    void setAmpThreshold(float t) { ampThresh_ = t; }

    void processTracks(const std::vector<Track> &tracks) {
        for (auto &t: tracks) {
            int   id       = t.id;
            float f        = t.freq, a = t.amp;
            int   voiceIdx = -1;
            for (int i = 0; i < voices_.size(); ++i) {
                Voice &v = voices_[i];
                if (v.trackId == id) {
                    voiceIdx = i;
                    break;
                }
            }

            if (voiceIdx == -1 && a > ampThresh_) {
                int vi = allocateVoice();
                if (vi >= 0) voices_[vi].noteOn(id, f, a);
            }
        }

        for (auto &v: voices_) {
            if (v.active) {
                bool stillThere = false;
                for (auto &t: tracks)
                    if (t.id == v.trackId) {
                        stillThere = true;
                        break;
                    }

                if (!stillThere)
                    v.noteOff(v.trackId);
            }
        }
    }

    float render(float in) {
        float mix = 0.0f;
        for (auto &v: voices_)
            mix += v.process(in);
        return mix;
    }

private:
    int allocateVoice() {
        for (int i = 0; i < maxVoices_; ++i)
            if (!voices_[i].active)
                return i;
        return -1;
    }

    const int maxVoices_;
    float     ampThresh_;

    std::vector<Voice> voices_;
};

#endif // MARTHA_H
