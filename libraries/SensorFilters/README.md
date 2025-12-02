# SensorFilters

This library provides classes that filter inputs to increase reliability and robustness.

## Classes

___

### DebounceToDigitalIn
Debounces a digital input

#### Implementation
Digital inputs are debounced by using a timer to sample the input at a high frequency (1ms), and only updates the debounced state of the pin when the signal changes for a set number of samples (_valid_read_count).

The signal must change for a time equal to the sampling interval multiplied by the required sample count, for example, with default values 1ms, and 5 required samples, a signal must change for 5ms straight to be registered.

#### Code Sample
```cpp
#include "mbed.h"
#include "debounce_to_digital_in.h"

//  Initializes a normal DigitalIn to pin PA_6.
DigitalIn button(PA_6);

//  Passes in a reference to the DigitalIn and sets the valid read count to 5.
//  The valid read count is simply the amount of different reads required in
//  a row for the debounced_in to return the different value. With our approach,
//  each DebounceToDigitalIn is rechecked every ms at the same time. This means
//  that the valid read count is the amount of ms that the signal needs to
//  change for a different value to be returned (HIGH -> LOW or LOW -> HIGH).
//  (See: https://my.eng.utah.edu/~cs5780/debouncing.pdf, Sections: Software
//  Debouncers-A Counting Algorithm)
DebounceToDigitalIn debounced_button(button, 5);

int main() {
    //  Sets the DebounceToDigitalIn object's valid read count to 7 (there must
    //  be a changed state for 7ms for the returned value of read() to change)
    debounced_button.set_valid_read_count(7);

    while (true) {
        //  Runs the default read() function on the DigitalIn.
        const bool default_button_state = button.read();

        //  Returns what the current debounced state is. Keep in mind that the
        //  read() does NOT run an update to the current debounced state since
        //  that happens at the same time for all objects every ms.
        const bool debounced_button_state = debounced_button.read();
    }

    return 0;
}
```

___

### SmoothToAnalogIn

#### Implementation
The SmoothToAnalogIn class implements an Exponential Weighted Moving Average filter to smooth out erratic analog inputs by applying a kind of digital low pass filter. 
The EWMA filter works according to the following formula:
<br />
<br />
<img width="932" height="106" alt="ewma-formula" src="https://github.com/user-attachments/assets/25deed81-95f4-4969-9351-fc398b5b2128" />
<br />
<br />
Where:
- Vsmoothed​(t) is the new smoothed value (_smoothed_value).
- Vraw​(t) is the current raw reading (raw_value from _analog_pin.read()).
- Vsmoothed​(t−Δt) is the previous smoothed value.
- α is the smoothing factor (or weight).

α, the smoothing factor is based a time constant (_time_constant) and time difference between readings. α is derived as follows:
<br />
<br />
<img width="264" height="98" alt="alpha-formula" src="https://github.com/user-attachments/assets/6711ed83-d87f-42c9-95c2-972c077f9b4d" />
<br />
<br />

This way, the cutoff frequency, which is used in determining the time constant as per the equation below, stays constant even if the time between reads is not consistent.
<br />
<br />
<img width="250" height="148" alt="time-constant-formula" src="https://github.com/user-attachments/assets/1460fb1d-6d14-4551-b960-6459aa9365ac" />
<br />
<br />

Here's a graph of a test run which shows how effective this method is at filtering noisy analog sensor input:
<br />
<br />
![ewma-graph-resize](https://github.com/user-attachments/assets/ff50976d-4ede-44f0-9e32-5e186e9443f4)

<br />

#### Other Methods
Two other options for smoothing are Rolling Average and Exponential Moving Average. 
<br />
<br />
<img width="550" height="300" alt="RA vs EMA vs EWMA" src="https://github.com/user-attachments/assets/5ce7f78b-eba7-496a-badc-cb864d6f9f4e" />
- Rolling Average: Yellow
    - Treats all data points within a fixed window equally.
    - Causes a lag where smoothed output is slow to react to a sudden change in sensor reading (step change).
  

- Exponential Moving Average: Blue
    - Uses an exponentially decreasing weighting factor ($\alpha$) for older observations. Recent data points contribute more significantly to the average than data points further in the past.
    - Reduces lag compared to the Rolling Average because it reacts faster to recent changes.


- Exponential Weighted Moving Average: Purple
  - The approach that the SmoothToAnalogIn objects use.
  - An exponential decaying weight is implemented to past data points. Same fundamental weighting scheme as the EMA.
  - Uses the same filtering as a First-Order Low-Pass Filter, where the $\alpha$ factor determines the filter's time constant or cutoff frequency.
      -  Low-Pass Filter: Allows low-frequency signals (the underlying trend) to pass through, while attenuating high-frequency signals (noise or rapid fluctuations).
##### Code Sample
```cpp
#include "mbed.h"
#include "smooth_to_analog_in.h"

//  Initializes a normal AnalogIn to pin PA_6.
AnalogIn in(PA_6);

//  Passes in a reference to the AnalogIn and sets the cutoff_frequency to 10Hz.
//  The cutoff frequency is similar to a first-pass RC filter, where the frequency
//  given is put at the -3db level.
//  (See: https://www.electronics-tutorials.ws/filter/filter_2.html, Sections: RC
//  Time Constant & Frequency Response of a 1st-order Low Pass Filter)
SmoothToAnalogIn smoothed_in(in, 10);

int main() {
    //  Sets the SmoothToAnalogIn object's cutoff frequency to 100Hz.
    smoothed_in.set_time_constant(100);
    //  Sets the referenced AnalogIn ("in" for this case) to have a
    //  reference_voltage of 3.3V.
    smoothed_in.set_reference_voltage(3.3);

    while (true) {
        //  Runs the default read_voltage() function on the AnalogIn.
        const float default_voltage = in.read_voltage();
        
        //  Runs the EWMA-smoothing algorithm on the current read value and returns
        //  the smoothed read as a result.
        const float smoothed_voltage = smoothed_in.read_voltage();
    }

    return 0;
}
```
  

___





