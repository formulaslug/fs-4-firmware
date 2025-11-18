# SensorFilters

This library provides classes that filter inputs to increase reliability and robustness.

## Classes

___

### DebounceToDigitalIn
Debounces a digital input

#### Code Sample
```TODO: put code here```

#### Implementation
Digital inputs are debounced by using a timer to sample the input at a high frequency (1ms), and only updates the debounced state of the pin when the signal changes for a set number of samples (_valid_read_count).

The signal must change for a time equal to the sampling interval multiplied by the required sample count, for example, with default values 1ms, and 5 required samples, a signal must change for 5ms to be registered.

___

### SmoothToAnalogIn

#### Code Sample
```TODO: put code here```

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

##### Other Methods
Two other options for smoothing are Rolling Average and Exponential Moving Average. 
- Rolling Average: Yellow
    - Treats all data points within a fixed window equally.
    - Causes a lag where smoothed output is slow to react to a sudden change in sensor reading (step change).
    - **Code Sample:** 
- Exponential Moving Average: Blue
    - Uses an exponentially decreasing weighting factor ($\alpha$) for older observations. Recent data points contribute more significantly to the average than data points further in the past.
    - Reduces lag compared to the Rolling Average because it reacts faster to recent changes.
    - **Code Sample:** 
- Exponential Weighted Moving Average: Purple
  - An exponential decaying weight is implemented to past data points. Same fundamental weighting scheme as the EMA.
  -  Giving it a direct physical meaning as a First-Order Low-Pass Filter, where the $\alpha$ factor determines the filter's time constant or cutoff frequency.
      -  Low-Pass Filter: Allows low-frequency signals (the underlying trend) to pass through, while attenuating high-frequency signals (noise or rapid fluctuations).
  -  **Code Sample:**
<br />
<br />
<img width="550" height="300" alt="RA vs EMA vs EWMA" src="https://github.com/user-attachments/assets/5ce7f78b-eba7-496a-badc-cb864d6f9f4e" />

___




