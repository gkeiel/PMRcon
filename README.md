# PMRcon

A lightweight **discrete-time proportional-multiple-resonant (PMR) control library for Arduino**.

The library implements PMR controllers using configurable **damping factors** and **harmonic frequencies**. It is designed for tracking periodic reference signals and rejecting periodic disturbances.

The library is intended as a practical reference for implementing, testing, and applying PMR control algorithms in embedded systems, particularly on **8-bit AVR microcontrollers** such as the ATmega328P. Furthermore, the sampling period is configured by the user and must match the actual interval between consecutive calls to the controller.


## PMR controller

The continuous-time PMR controller is defined by:

$$C_{PMR}(s) = k_p + \sum_{h~\in~\mathcal{H}} \frac{k_{r_h} s}{s^2 + 2\xi\omega_h s + \omega_h^2}$$

where:
- $k_p$ is the proportional gain;
- $k_{r_h}$ is the resonant gain of the h-th mode;
- $\xi_h$ is the damping factor of h-th mode;
- $\omega_h$ is the angular frequency of the h-th mode, defined as $\omega_h = h\omega_0$;
- $\omega_0$ is the fundamental angular frequency;
- $\mathcal{H}$ is the set of configured harmonic numbers, supporting up to 10 resonant modes.

The discrete-time controller is obtained by applying the Tustin discretization with prewarping to each resonant mode of the continuous-time transfer function.


## Features

- **PMR control**: Implements discrete-time PMR action.
- **Multiple resonant modes**: Supports multiple modes at configurable harmonic frequencies.
- **Configurable damping**: Allows adjustment of the damping factor for each resonant mode, with zero as default.
- **Parameter configuration**: Provides methods to adjust individual PMR gains and additional parameters.


## Installation

Download or clone this repository and place the PMRcon folder inside the Arduino libraries directory. Alternatively, install the library through the Arduino IDE using **Sketch → Include Library → Add .ZIP Library**.

## Basic usage

Create a PR controller by specifying its proportional gain, resonant gain, sampling period, and fundamental angular frequency:

```cpp
#include <PMRcon.h>

float k_p = 10;  
float k_r = 5;  
float T_s = 0.01;  
float w_0 = 2.0f*PI*50;

PMR pmr(k_p, k_r, T_s, w_0);
```

Although the constructor uses a single resonant mode by default, with a zero damping factor, the number of modes, resonant gains and damping factor can also be configured individually:
```cpp
const int harmonics[] = {1, 3, 5};
const float k_r[] = {10, 8, 4};
const float xi[]  = {0, 0.001, 0.001};

PMR pmr(k_p, k_r, T_s, w_0, xi, harmonics, 3);
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
| `get_kr(i)` | Returns the resonant gain of the specified mode. |
| `get_xi(i)` | Returns the damping factor of the specified mode. |
| `get_modes()` | Returns the number of configured modes. |


## Support the Project

Future versions may include improvements and additional features. 

If you find this library useful, consider making a voluntary contribution solely to support this project's maintenance and further development. 

[![Support the Project](https://img.shields.io/badge/Support%20the%20Project-Ko--fi-FF5E5B?logo=ko-fi&logoColor=white)](https://ko-fi.com/gkeiel)