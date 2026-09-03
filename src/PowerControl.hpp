#pragma once

#include <stdint.h>

class PowerControl {
private:
    float currentDuty = 1500.0f;
    float targetDuty  = 3000.0f;
    float rampRate    = 0.0f;
    bool  ramping     = false;

public:
    void startRamp(float startDuty, float endDuty, float durationSec) {
        currentDuty = startDuty;
        targetDuty  = endDuty;
        rampRate    = (durationSec > 0.0f) ? ((endDuty - startDuty) / durationSec) : 0.0f;
        ramping     = true;
    }

    float computeRampDuty(float dt) {
        if (ramping) {
            currentDuty += rampRate * dt;
            if ((rampRate >= 0.0f && currentDuty >= targetDuty) ||
                (rampRate < 0.0f && currentDuty <= targetDuty)) {
                currentDuty = targetDuty;
                ramping     = false;
            }
        }
        return currentDuty;
    }

    void reset() {
        currentDuty = 1500.0f;
        targetDuty  = 1500.0f;
        rampRate    = 0.0f;
        ramping     = false;
    }
};
