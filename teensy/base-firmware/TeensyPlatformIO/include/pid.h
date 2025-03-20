#include <vector>

class PIDController {
public:
    PIDController(float PP, float PD, float PDD, float dt)
    : PP(PP), PD(PD), PDD(PDD), dt(dt), x(0.0f), v_prev(0.0f) {}

    // 'v' is the current velocity measurement (audio input sample).
    float processSample(float v) {
        x += v * dt;
        float a = (v - v_prev) / dt;
        v_prev = v;
        // u = PDD * acceleration + PD * velocity + PP * displacement.
        float u = PDD * a + PD * v + PP * x;
        return u;
    }

    void reset() {
        x = 0.0f;
        v_prev = 0.0f;
    }

private:
    float PP;   // Proportional gain (affects displacement feedback)
    float PD;   // Derivative (velocity) gain (affects decay time)
    float PDD;  // "Double derivative" gain (affects both resonance frequency and decay time)
    float dt;   // Sampling interval
    float x;    // Integrated displacement
    float v_prev; // Previous velocity sample, for numerical differentiation
};
