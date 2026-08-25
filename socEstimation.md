# Project description and methods
by Emma Su

The goal of this project is to report the SOC of our battery cells to the dash. Because SOC estimation can be tricky, one way it can be done is with a discrete Kalman filter.

### Filter overview

The Kalman filter is useful because it can minimize estimation errors pretty quickly and with not much information. The dynamics that are to be estimated are contained in the state vector. The previous estimates at k-1 are propagated forward by the state equations as the initial state estimates at k. Then the initial estimates are corrected with the gain calculated from the error between the measurements and the initial estimates, to get the corrected estimates. The equations involved and their descriptions are as follows.

System model: x<sub>k</sub> = f(x<sub>k-1</sub>) + w<sub>k</sub>  
Measurement model: y<sub>k</sub> = h(x<sub>k</sub>) + v<sub>k</sub>  
Initial estimate (state propagation): x<sup>-</sup><sub>k</sub> = f(x<sup>+</sup><sub>k-1</sub>)  
Initial error covariance (covariance propagation): P<sup>-</sup><sub>k</sub> = A<sub>k</sub>P<sup>+</sup><sub>k-1</sub>A<sub>k</sub><sup>T</sup> + Q   
Gain: K<sub>k</sub> = P<sup>-</sup><sub>k</sub>H<sub>k</sub><sup>T</sup>(H<sub>k</sub>P<sup>-</sup><sub>k</sub>H<sub>k</sub><sup>T</sup> + R)<sup>-1</sup>  
Estimate update: x<sup>+</sup><sub>k</sub> = x<sup>-</sup><sub>k</sub> + K<sub>k</sub>\[z<sub>k</sub> - h(x<sup>-</sup><sub>k</sub>)\]  
Covariance update: P<sup>+</sup><sub>k</sub> = \[I - K<sub>k</sub>H<sub>k</sub>\]P<sup>-</sup><sub>k</sub>  
  
A<sub>k</sub> = $\frac{df(x)}{dx}$  
H<sub>k</sub> = $\frac{dh(x)}{dx}$  

w and v are uncertainties. Q is a matrix of the process uncertainties and R is a matrix of the measurement uncertainties. Measurements are the data we have. Uncertainty means, for example, if the measurements are assumed to be taken every 10 ms, the measurement uncertainty can be calculated by finding the variance of the actual time intervals in the collected data. The + and - are used to indicate intial and updated estimates. Lastly, the filter equations are for a nonlinear system but work for a linear system too.  

### What the filter looks like for SOC estimation

We define SOC as the releasable capacity over the nominal capacity. SOC can be calculated by subtracting the depth of discharge (DOD) from the state of health (SOH). We define DOD as the released capacity over the nominal capacity and the SOH as releasable capacity plus released capacity, or the total capacity over the nominal capacity. The DOD can be calculated with Coulomb counting. The SOH is more difficult because it is not always 1 as the cell degrades. In one example of SOC estimation I found online, the SOH is corrected with an ISR at the fully charged/discharged states to SOH -> SOC - DOD. The fully charged/discharged states might not occur frequently enough for our cells, and we want the estimation to work pretty well across different uses (ie without conditions specified with an interrupt). One way to apply the Kalman filter to solve this issue is to make the SOH a part of the state vector and define the SOH equation as SOH(k) = SOH(k-1) + some uncertainty. The state vector can also be populated by things we have measurements of such as current. Then the SOC can be calculated by subtracting the DOD calculated from the updated current estimates, from the updated SOH estimate.  

The filter needs initial values at the initiation of the filter as well as data to compare the estimates against. The data is read from parquet files with Python libraries and converted to C using the Python/C API.  

Some questions (not necessarily all to be answered immediately but any feedback is welcome too): I am not sure how to actually see the accuracy of the SOH estimations. Right now to me there isn't a lot of information about SOH (like an initial estimate of the variance) that can be injected into the filter to make it more accurate. Also, how the estimation will end up on the dash I don't know yet either. I just want to make the estimation work for now.
