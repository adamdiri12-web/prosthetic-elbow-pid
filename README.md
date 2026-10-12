# PID-Controlled Prosthetic Elbow

A self-contained C++17 simulation of a feedback-controlled prosthetic elbow joint. The project models a torque-driven second-order elbow plant, potentiometer angle sensing with filtering/noise, actuator saturation, and a PID controller with anti-windup.

It is designed as an approachable control-systems portfolio project: it produces numerical step-response data, variable-load robustness tests, and open-loop Bode, Nyquist, and root-locus data without requiring MATLAB, Python, or third-party libraries.

## Highlights

- Closed-loop elbow-angle tracking to a 55° step target
- PID tuning comparison between a baseline and a damped tuned controller; the included nominal simulation yields approximately 35% lower overshoot and 20% lower settling time
- Potentiometer sensor model with first-order filtering and reproducible measurement noise
- Torque saturation and conditional-integration anti-windup
- Load sweep at 80%, 100%, and 125% nominal inertia
- CSV output for step response, Bode, Nyquist, root locus, and performance metrics

## Build and run

Requirements: CMake 3.20+ and a C++17 compiler.

```powershell
cmake -S . -B build
cmake --build build --config Release
.\build\Release\prosthetic_joint_pid.exe results
```

On non-multi-config CMake generators, run `./build/prosthetic_joint_pid results` instead.

## Output files

| File | Purpose |
| --- | --- |
| `step_response_baseline.csv` | Initial PID response for comparison |
| `step_response_tuned_nominal.csv` | Tuned nominal-load response |
| `step_response_load_80/100/125.csv` | Robustness under changing inertia |
| `bode_open_loop.csv` | Open-loop magnitude and phase response |
| `nyquist_open_loop.csv` | Complex open-loop response coordinates |
| `root_locus.csv` | Closed-loop pole trajectories as gain changes |
| `metrics_summary.csv` | Overshoot, rise time, settling time, final error |

Plot any CSV in Excel, MATLAB, Python, or a plotting tool of your choice. For root locus, plot `real` against `imaginary` and color/group by `root_index`.

## Modeling notes

The joint model is

`I θ¨ + b θ˙ = τ`

where `I` represents reflected inertia, `b` is viscous damping, and `τ` is motor torque. The nominal inertia is 0.045 kg·m², and the controller has a ±6 N·m torque limit. The Bode/Nyquist calculation uses the linearized plant `1 / (I s² + b s)` and the continuous PID transfer function.

This is an educational simulation, not a medical-device controller or safety-validated design.

## Resume wording

**PID-Controlled Prosthetic Elbow | C++17, CMake**

- Built a closed-loop prosthetic elbow simulator with a second-order joint model, filtered potentiometer feedback, actuator saturation, and anti-windup PID control.
- Tuned and benchmarked PID gains using automated step-response metrics; evaluated overshoot, rise/settling time, and steady-state error.
- Generated Bode, Nyquist, root-locus, and variable-inertia load-sweep datasets to assess stability and robustness.
