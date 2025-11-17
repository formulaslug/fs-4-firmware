# SensorFilters

This library provides classes that filter inputs to increase reliability and robustness.

## Classes

___

### DebounceToDigitalIn
Debounces a digital input

#### Code sample
```TODO: put code here```

#### Implementation
Digital inputs are debounced by using a timer to sample the input at a high frequency (1ms), and only updates the debounced state of the pin when the signal changes for a set number of samples (_valid_read_count).

The signal must change for a time equal to the sampling interval multiplied by the required sample count, for example, with default values 1ms, and 5 required samples, a signal must change for 5ms to be registered.

___

### SmoothToAnalogIn

#### Code sample
```TODO: put code here```

### Implementation
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

___
