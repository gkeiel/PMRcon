# PMRcon

A lightweight **discrete-time proportional-multiple-resonant (PMR) control library for Arduino**.

The library implements PMR controllers using configurable **damping factors** and **harmonic frequencies**. It is designed for tracking periodic reference signals and rejecting periodic disturbances.

The library is intended as a practical reference for implementing, testing, and applying PMR control algorithms in embedded systems, particularly on **8-bit AVR microcontrollers** such as the ATmega328P. Furthermore, the sampling period is configured by the user and must match the actual interval between consecutive calls to the controller.


## PMR controller

The continuous-time PMR controller is defined by:

$$C_{PMR}(s) = k_p + \sum_{h~\in~\mathcal{H}} \frac{k_r s}{s^2 + 2\xi\omega_h s + \omega_h^2}$$

where:
- $k_p$ is the proportional gain;
- $k_r$ is the resonant gain, common to all modes;
- $\xi$ is the damping factor, common to all modes;
- $\omega_h$ is the angular frequency of the h-th resonant mode, defined as $\omega_h = h\omega_0$;
- $\omega_0$ is the fundamental angular frequency;
- $\mathcal{H}$ is the set of configured harmonic numbers, supporting up to 10 resonant modes.

The discrete-time controllers are obtained by applying the Tustin discretization with prewarping to each mode of the continuous-time transfer function.


## Features

- **PMR control**: Implements discrete-time PMR action.
- **Multiple resonant modes**: Supports multiple modes at configurable harmonic frequencies.
- **Configurable damping**: Allows adjustment of the damping factor for all resonant modes.
- **Parameter configuration**: Provides methods to adjust PMR gains and additional parameters.


## Installation

Download or clone this repository and place the PMRcon folder inside the Arduino libraries directory. Alternatively, install the library through the Arduino IDE using **Sketch → Include Library → Add .ZIP Library**.

## Basic usage

Create a PMR controller by specifying its proportional gain, resonant gain, sampling period, and fundamental angular frequency:

```cpp
#include <PMRcon.h>

float k_p = 10.0f;  
float k_r = 5.0f;  
float T_s = 0.01f;  
float w_0 = 2.0f*PI*50.0f;

PMR pmr(k_p, k_r, T_s, w_0);
```

Although the constructor uses a single resonant mode by default, with a zero damping factor, the number of resonant modes and a global damping factor can also be configured explicitly:
```cpp

float xi = 0.001f;                    // damping factor
const int harmonics[] = {1, 3, 5, 7}; // resonant modes at the harmonics

PMR pmr(k_p, k_r, T_s, w_0, xi, harmonics, 4);
```

To calculate the control signal, call compute() with the reference and measured output:
```cpp
float u = pmr.compute(r, y);
```

## Configuration

The controller provides the following methods:

| Method | Description |
|---|---|
| `set_tuning(k_p, k_r)` | Sets the PMR gains. |
| `set_limit(u_min, u_max)` | Sets the output saturation limits. |
| `compute(r, y)` | Computes the control signal. |
| `reset()` | Resets the controller's internal states. |
| `get_kp()` | Returns the proportional gain. |
| `get_kr()` | Returns the resonant gain. |
| `get_modes()` | Returns the number of configured resonant modes. |


## Support the Project

Future versions may include improvements and additional features. 

If you find this library useful, consider making a voluntary contribution solely to support this project's maintenance and further development. 

[![Support the Project](https://img.shields.io/badge/Support%20the%20Project-Ko--fi-FF5E5B?logo=ko-fi&logoColor=white)](https://ko-fi.com/gkeiel)