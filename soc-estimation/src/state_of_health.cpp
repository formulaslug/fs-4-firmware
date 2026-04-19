// ------------------------------------------------------------------------------------------------
// State of health estimation
// Implementation File
// ------------------------------------------------------------------------------------------------

#include "state_of_health.h"

// Function Implementations -----------------------------------------------------------------------
float sohEstimate(float lastSOH, float currentDraw, float secondsSinceLastCall) {
    // get c rate
    float cRate = currentDraw / DESIGNATED_CAPACITY;
    // get c rate slope (we can do this because the relationship is linear)
    float cRateSlope = cRate * C_RATE_GRADIENT;
    // get t rate
    float tRate = (72 * cRateSlope * ROUND_TRIP_EFFICIENCY * lastSOH) / (cRate * C1_SLOPE_EQUIVALENT);
    // turn into eq cycles
    float eqCycles = (secondsSinceLastCall)/tRate;
    // turn into soh using linear approximation
    float newSOH = eqCycles * C1_SLOPE_EQUIVALENT
}