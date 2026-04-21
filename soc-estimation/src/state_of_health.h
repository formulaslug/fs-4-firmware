// ------------------------------------------------------------------------------------------------
// State of health estimation
// Header File
// ------------------------------------------------------------------------------------------------

#ifndef STATE_OF_HEALTH
#define STATE_OF_HEALTH

const float DESIGNATED_CAPACITY = 3000;     // mAh - capacity from manufacturer
const float ROUND_TRIP_EFFICIENCY = 1;      // unitless - charging and discharging efficiency
const float C_RATE_GRADIENT = 0.004367;     // coefficient of slope of SOH / C-Rate with respect to C-Rate SOH/(C-Rate^2)
const float C_RATE_OFFSET = 0.021108;       // intercept of slope gradient line through data points
const float C1_SLOPE_EQUIVALENT = 0.025475; // slope of SOH vs. # of cycles at 1C
// 0.063475 @ ~9.7C
// 0.025475 @ 1C
// 0.004367 = SOH/(C-Rate)^2

// Function Prototypes ----------------------------------------------------------------------------

// sohEstimate()
// call every so to keep SOH updated, returns SOH % as a decimal
float sohEstimate(float lastSOHPercent, float currentDraw, float milliSinceLastCall);

#endif
