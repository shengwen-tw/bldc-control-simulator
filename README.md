## BLDC control simulator

Simulation of different BLDC control algorithms written in MATLAB.

## Currently implemented algorithms

**1. Square-Wave Commutation Control (Robust and easier for implementation)**
* PI speed control
* Hysteresis current control
* Trapezoidal back EMF model
* Sensing of rotor position sector with back EMF 

**2. Field-Oriented Control (Has better efficiency and lower noise sound in theory)**
* PI speed control
* DQ current control with PI control and SVPWM
* Sinusoidal back EMF model
* Assumed the rotor position is known

## Run simulation

Execute **main.m**

## References

1. [Modeling, Simulation, and Analysis of Permanent-Magnet Motor Drives, Part I](https://ieeexplore.ieee.org/document/25541)
2. [Modeling, Simulation, and Analysis of Permanent-Magnet Motor Drives, Part II](https://ieeexplore.ieee.org/document/25542)
3. [Electric Motor Control: DC, AC, and BLDC Motors](https://www.elsevier.com/books/electric-motor-control/kim/978-0-12-812138-2)
