# FS-5's Traction Control

FS-5's Traction Control (TC) reduces torque to prevent wheel spin during acceleration. FS-5 is RWD, so it compares the front wheel speeds with the rear wheel speeds, determining longitudinal slip and regulating torque reduction through a filtered PID loop.

## Tuning Constants

**KP, KD, KI**
> KP (Proportional gain). Reacts to current slip error. 
> KI (Integral gain). Accumulated slip error
> KD (Derivative gain). Adjusts for a sudden wheel acceleration
> These values need track testing to be tuned. Baseline values will be researched and determined before then.

**MAX_INTEGRATOR_REDUCTION_FACTOR**
> Currently 0.5. KP and KD should be tuned first, then this should be adjusted accordingly.
>> 0.5 means the integral component contributes 50% to the torque adjusting, which should be a good baseline. 

**TARGET_WHEEL_SLIP**
> Currently 0.09, which is acceptable for Hoosier tires. Need to check wheel datasheet

**ACTIVATION_RPM**
> Currently 100 RPM, which is ~ 7 mph. Should be reasonable, it (potentially) could be lower.

**FILTER_TIME_CONSTANT**
> Currently 0.2s. Used in the low-pass filter for smoothing out the raw derivative. This should be lower.

**MIN_OUTPUT**
> Minimum torque reduction (0.2)

**MAX_OUTPUT**
> Maximum torque reduction (1.0)


 
