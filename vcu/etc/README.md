# FS-5's Traction Control

FS-5's Traction Control (TC) reduces torque to prevent wheel spin during acceleration. FS-5 is RWD, so it compares the front wheel speeds with the rear wheel speeds, determining longitudinal slip and regulating torque reduction through a filtered PID loop.

## Tuning Constants

**KP, KD, KI**
> KP (Proportional gain). Reacts to current slip error. 
> KI (Integral gain). Accumulated slip error
> KD (Derivative gain). Adjusts for a sudden wheel acceleration
> These values need track testing to be tuned. Baseline values will be researched and determined before then.
> KP should be tuned first, with KI and KD set to 0. A good baseline should be 2.0, corresponding with our minimum torque reduction.


**MAX_INTEGRATOR_REDUCTION_FACTOR**
> To be determined following testing

**TARGET_WHEEL_SLIP**
> Currently 0.09, which is acceptable for Hoosier tires.
> [This]("https://www.researchgate.net/publication/228731408_Using_Timber_in_a_Multi-Body_Design_Environment_to_Develop_Reliable_Embedded_Software#pf5") source contains a graph comparing longitiduinal force to slip ratio, which reveals the ideal slip ratio to be 0.1.
> [Graph]("https://www.researchgate.net/figure/Slip-ratio-curves-in-different-conditions-for-a-Formula-SAE-tire-18_fig2_228731408")


**ACTIVATION_RPM**
> Currently 100 RPM, which is ~ 7 mph. Should be reasonable, it (potentially) could be lower.

**FILTER_TIME_CONSTANT**
> Currently 0.2s. Used in the low-pass filter for smoothing out the raw derivative. This should be lower.

**MIN_OUTPUT**
> Minimum torque reduction (0.2). Anywhere from 0.1 to 0.3 should be fine. 0.4 and above would bottleneck the maximum torque reduction to < 0.6, which could be unsafe.

**MAX_OUTPUT**
> Maximum torque reduction (1.0)


 
