# Project summary

## Problem

Prosthetic joints need smooth, repeatable position control despite changing forearm loads and imperfect sensor data. This project evaluates a classical PID position loop for an elbow-joint model with a potentiometer-style feedback path.

## Implementation

The C++ program simulates a 55° elbow flexion command through a second-order torque plant. It includes a filtered/noisy sensor reading, torque saturation, derivative-on-measurement, and conditional integration to reduce integral windup. A baseline controller and a tuned controller are measured using the same step-response criteria.

## Analyses produced

- Time-domain: angle tracking, sensor output, controller torque, rise time, settling time, overshoot, and steady-state error.
- Robustness: three reflected-inertia cases, representing light, nominal, and heavy loads.
- Frequency-domain: Bode magnitude/phase data and Nyquist coordinates for the linearized loop.
- Stability sensitivity: root-locus pole locations while the controller gain is swept.

## What to discuss in an interview

The controller uses derivative action on the measured angle instead of on the error; this avoids a torque spike when the reference changes. The integrator is only updated when the torque is unsaturated (or when its error is driving it back toward range), preventing windup. The frequency-domain outputs complement the time-domain plots: they expose stability margin and pole migration that may not be obvious from a single step response.

## Resume entry

**PID-Controlled Prosthetic Elbow — C++17, CMake**

Built a C++ simulation of a prosthetic elbow position loop using PID control, filtered potentiometer feedback, actuator saturation, and anti-windup logic. Automated step-response benchmarking and generated Bode, Nyquist, root-locus, and variable-inertia load-sweep data to evaluate stability and control robustness.
