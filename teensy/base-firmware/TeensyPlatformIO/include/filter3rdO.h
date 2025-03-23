#ifndef FILTER3RDO_H
#define FILTER3RDO_H

#include <array>

class ThirdOrderFilter
{
public:
    ThirdOrderFilter(float fs) : fs(fs) {};
    void setCoefficients(float a1, float a2, float b0, float b1, float b2);
    float process(float input);
    void setFilterParams(float fc, float Q);
    std::array<float, 5> getCoefficients() const;
    float getQ() const { return Q; };
    float getFc() const { return fc; };
private:
    float a1, a2, b0, b1, b2;
    float x[2] = {0.0f, 0.0f};
    float y[2] = {0.0f, 0.0f};
    float fs;
    float Q, fc;
};

#endif // FILTER3RDO_H

void ThirdOrderFilter::setCoefficients(float a1, float a2, float b0, float b1, float b2)
{
    this->a1 = a1;
    this->a2 = a2;
    this->b0 = b0;
    this->b1 = b1;
    this->b2 = b2;
}

float ThirdOrderFilter::process(float input)
{
    float out = b0 * input + b1 * x[0] + b2 * x[1] - a1 * y[0] - a2 * y[1];
    x[1] = x[0];
    x[0] = input;
    y[1] = y[0];
    y[0] = out;
    return out;
}

void ThirdOrderFilter::setFilterParams(float fc, float Q)
{
    float K0 = 1.0; // gain factor
    this->fc = fc;
    this->Q = Q;
    // Analog filter coefficients:
    double a1_analog = (2.0 * M_PI * fc) / Q;
    double a2_analog = std::pow(2.0 * M_PI * fc, 2);
    double b0_analog = K0 * (2.0 * M_PI * fc) / Q;
    double T = 1.0 / fs;
    double K = 2.0 / T; // equivalently, 2*fs
    double A = K*K + a1_analog*K + a2_analog;           // coefficient for z^2
    double B = -2*K*K + 2*a2_analog;                      // coefficient for z^1
    double C = K*K - a1_analog*K + a2_analog;             // coefficient for z^0

    b0 = (b0_analog * K) / A;  // coefficient for z^2 term
    b1 = 0.0;                  // coefficient for z^1 term (remains 0)
    b2 = -(b0_analog * K) / A; // coefficient for z^0 term

    // Digital denominator (normalized so a0 is 1):
    a1 = B / A;
    a2 = C / A;
}

std::array<float, 5> ThirdOrderFilter::getCoefficients() const
{
    return {a1, a2, b0, b1, b2};
}