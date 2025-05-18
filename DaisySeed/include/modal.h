//
// Created by Levin on 17.05.25.
//

#ifndef MODAL_H
#define MODAL_H

struct ModalMode {
    float f0;
    float Q;
};

struct ModalFilter {
    int       numModes;
    ModalMode model[];
};

struct Modal {
    double r,      cosθ, gain;
    double y1 = 0, y2 = 0;

    void init(double f0, double Q, double fs) {
        r        = exp(-M_PI * f0 / (Q * fs));
        double θ = 2 * M_PI * f0 / fs;
        cosθ     = cos(θ);
        // this ensures unity gain at f0:
        gain = (1.0 - r);
    }

    float process(float x) {
        // y[n] = 2⋅r⋅cosθ⋅y[n-1] − r²⋅y[n-2] + gain⋅x[n]
        double y0 = 2.0 * r * cosθ * y1
                    - r * r * y2
                    + gain * x;
        y2 = y1;
        y1 = y0;
        return (float) y0;
    }
};

static constexpr ModalFilter BANJO_FILTER{
    42,
    {
        {310.0033, 15.0000},
        {711.5869, 20.0000},
        {1115.5421, 25.0000},
        {493.9408, 20.0000},
        {904.3712, 25.0000},
        {1311.4501, 30.0000},
        {662.0272, 25.0000},
        {1085.0573, 30.0000},
        {1497.9004, 35.0000},
        {822.4593, 30.0000},
        {1258.2823, 35.0000},
        {1677.7745, 40.0000},
        {978.2045, 35.0000},
        {1426.3390, 40.0000},
        {1852.7472, 45.0000},
        {1130.7219, 40.0000},
        {1590.5553, 45.0000},
        {2023.8914, 50.0000},
        {1280.8524, 45.0000},
        {1751.7798, 50.0000},
        {2191.9428, 55.0000},
        {1429.1312, 50.0000},
        {1910.5927, 55.0000},
        {2357.4313, 60.0000},
        {1575.9226, 55.0000},
        {2067.4110, 60.0000},
        {2520.7528, 65.0000},
        {1721.4875, 60.0000},
        {2222.5459, 65.0000},
        {2682.2126, 70.0000},
        {1866.0201, 65.0000},
        {2376.2366, 70.0000},
        {2842.0515, 75.0000},
        {2009.6693, 70.0000},
        {2528.6717, 75.0000},
        {3000.4635, 80.0000},
        {2152.5522, 75.0000},
        {2680.0029, 80.0000},
        {3157.6074, 85.0000},
        {2294.7625, 80.0000},
        {3313.6152, 90.0000}
    }
};

struct ModalBPFilter {
    BiQuad    filters[BANJO_FILTER.numModes];
    void init() {
        for (int i = 0; i< BANJO_FILTER.numModes; i++) {
            filters[i] = BiQuad();
            filters[i].setFilterParams(BANJO_FILTER.model[i].f0, BANJO_FILTER.model[i].Q);
        }
    }

    float process(float in) {
        float sum = 0;
        for (int i = 0; i < BANJO_FILTER.numModes; i++) {
            sum += filters[i].process(in);
        }
        return sum;
    }

};

#endif // MODAL_H
