#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;
constexpr double kPi = 3.14159265358979323846;

struct PidGains { double kp, ki, kd; };
struct Metrics {
  double rise_time_s = std::numeric_limits<double>::quiet_NaN();
  double settling_time_s = std::numeric_limits<double>::quiet_NaN();
  double overshoot_pct = 0.0;
  double steady_state_error_deg = 0.0;
};
struct Sample { double time, reference, angle, measured, torque, load; };

// Plant: I*q'' + b*q' = torque. Angles are kept internally in radians.
class ProstheticElbow {
 public:
  ProstheticElbow(double inertia, double damping) : inertia_(inertia), damping_(damping) {}
  void step(double torque, double dt) {
    const double acceleration = (torque - damping_ * velocity_) / inertia_;
    velocity_ += acceleration * dt;
    angle_ += velocity_ * dt;
  }
  [[nodiscard]] double angle() const { return angle_; }
  [[nodiscard]] double velocity() const { return velocity_; }
 private:
  double inertia_, damping_;
  double angle_ = 0.0, velocity_ = 0.0;
};

class Potentiometer {
 public:
  explicit Potentiometer(double time_constant_s) : tau_(time_constant_s) {}
  double sample(double actual_angle, double dt) {
    // Deterministic low-amplitude noise keeps every run reproducible.
    const double noise = (0.18 * std::sin(71.0 * clock_) + 0.08 * std::sin(193.0 * clock_)) * kPi / 180.0;
    filtered_ += (actual_angle + noise - filtered_) * dt / tau_;
    clock_ += dt;
    return filtered_;
  }
 private:
  double tau_, filtered_ = 0.0, clock_ = 0.0;
};

class PID {
 public:
  PID(PidGains gains, double torque_limit) : gains_(gains), limit_(torque_limit) {}
  double update(double setpoint, double measurement, double dt) {
    const double error = setpoint - measurement;
    // Derivative-on-measurement avoids a derivative kick at a reference step.
    const double derivative = -(measurement - previous_measurement_) / dt;
    const double candidate_integral = integral_ + error * dt;
    const double unsaturated = gains_.kp * error + gains_.ki * candidate_integral + gains_.kd * derivative;
    const double saturated = std::clamp(unsaturated, -limit_, limit_);
    // Conditional integration is a simple anti-windup mechanism.
    if (std::abs(unsaturated - saturated) < 1e-12 || error * unsaturated < 0.0)
      integral_ = candidate_integral;
    previous_measurement_ = measurement;
    return saturated;
  }
 private:
  PidGains gains_;
  double limit_, integral_ = 0.0, previous_measurement_ = 0.0;
};

double deg(double radians) { return radians * 180.0 / kPi; }
double rad(double degrees) { return degrees * kPi / 180.0; }

Metrics calculate_metrics(const std::vector<Sample>& data, double target) {
  Metrics m;
  const double low = 0.1 * target, high = 0.9 * target;
  double t10 = -1.0;
  for (const auto& p : data) {
    if (t10 < 0.0 && p.angle >= low) t10 = p.time;
    if (t10 >= 0.0 && p.angle >= high) { m.rise_time_s = p.time - t10; break; }
  }
  double peak = 0.0;
  for (const auto& p : data) peak = std::max(peak, p.angle);
  m.overshoot_pct = std::max(0.0, (peak - target) / target * 100.0);
  const double band = 0.02 * std::abs(target);
  for (size_t i = 0; i < data.size(); ++i) {
    bool stays_in_band = true;
    for (size_t j = i; j < data.size(); ++j) {
      if (std::abs(data[j].angle - target) > band) { stays_in_band = false; break; }
    }
    if (stays_in_band) { m.settling_time_s = data[i].time; break; }
  }
  m.steady_state_error_deg = deg(target - data.back().angle);
  return m;
}

std::vector<Sample> simulate(PidGains gains, double load_factor, double duration_s = 3.0) {
  constexpr double dt = 0.001;
  constexpr double nominal_inertia = 0.045; // kg m^2, representative elbow + forearm load
  constexpr double damping = 0.11;          // N m s/rad
  const double target = rad(55.0);
  ProstheticElbow joint(nominal_inertia * load_factor, damping);
  Potentiometer sensor(0.015);
  PID controller(gains, 6.0);
  std::vector<Sample> data;
  data.reserve(static_cast<size_t>(duration_s / dt) + 1);
  for (double t = 0.0; t <= duration_s + 1e-12; t += dt) {
    const double measurement = sensor.sample(joint.angle(), dt);
    const double torque = controller.update(target, measurement, dt);
    joint.step(torque, dt);
    data.push_back({t, deg(target), deg(joint.angle()), deg(measurement), torque, load_factor});
  }
  return data;
}

void write_step_csv(const fs::path& path, const std::vector<Sample>& data) {
  std::ofstream out(path);
  out << "time_s,reference_deg,angle_deg,measured_angle_deg,torque_nm,load_factor\n";
  out << std::fixed << std::setprecision(6);
  for (const auto& p : data)
    out << p.time << ',' << p.reference << ',' << p.angle << ',' << p.measured << ',' << p.torque << ',' << p.load << '\n';
}

std::array<std::complex<double>, 3> cubic_roots(double a, double b, double c, double d) {
  // Durand-Kerner iteration for a*x^3+b*x^2+c*x+d = 0.
  b /= a; c /= a; d /= a;
  std::array<std::complex<double>, 3> z = {std::complex<double>(1, 0), std::complex<double>(-0.5, 0.9), std::complex<double>(-0.5, -0.9)};
  auto f = [=](std::complex<double> x) { return ((x + b) * x + c) * x + d; };
  for (int iter = 0; iter < 200; ++iter) {
    double change = 0.0;
    for (size_t i = 0; i < z.size(); ++i) {
      std::complex<double> denom(1.0, 0.0);
      for (size_t j = 0; j < z.size(); ++j) if (i != j) denom *= (z[i] - z[j]);
      const auto delta = f(z[i]) / denom;
      z[i] -= delta;
      change = std::max(change, std::abs(delta));
    }
    if (change < 1e-12) break;
  }
  return z;
}

void write_frequency_data(const fs::path& output_dir, PidGains g) {
  std::ofstream bode(output_dir / "bode_open_loop.csv");
  std::ofstream nyquist(output_dir / "nyquist_open_loop.csv");
  bode << "frequency_rad_s,magnitude_db,phase_deg\n";
  nyquist << "frequency_rad_s,real,imaginary\n";
  constexpr double inertia = 0.045, damping = 0.11;
  for (int i = 0; i <= 800; ++i) {
    const double w = std::pow(10.0, -1.0 + 4.0 * i / 800.0);
    const std::complex<double> s(0.0, w);
    const std::complex<double> controller = g.kp + g.ki / s + g.kd * s;
    const std::complex<double> plant = 1.0 / (inertia * s * s + damping * s);
    const auto loop = controller * plant;
    bode << w << ',' << 20.0 * std::log10(std::abs(loop)) << ',' << std::arg(loop) * 180.0 / kPi << '\n';
    nyquist << w << ',' << loop.real() << ',' << loop.imag() << '\n';
  }
}

void write_root_locus(const fs::path& path, PidGains g) {
  std::ofstream out(path);
  out << "gain,root_index,real,imaginary\n";
  constexpr double inertia = 0.045, damping = 0.11;
  for (int i = 0; i <= 300; ++i) {
    const double gain = 0.02 + 3.0 * i / 300.0;
    const auto roots = cubic_roots(inertia, damping + gain * g.kd, gain * g.kp, gain * g.ki);
    for (size_t k = 0; k < roots.size(); ++k)
      out << gain << ',' << k << ',' << roots[k].real() << ',' << roots[k].imag() << '\n';
  }
}

void print_metrics(const std::string& name, const Metrics& m) {
  std::cout << std::fixed << std::setprecision(2)
            << name << ": overshoot=" << m.overshoot_pct << "% | rise=" << m.rise_time_s
            << " s | settling=" << m.settling_time_s << " s | final error=" << m.steady_state_error_deg << " deg\n";
}

int main(int argc, char* argv[]) {
  const fs::path output_dir = argc > 1 ? fs::path(argv[1]) : fs::path("results");
  fs::create_directories(output_dir);

  // Baseline values represent an initial, under-damped hand-tuning.
  const PidGains baseline{5.0, 8.0, 0.50};
  // Tuned values add derivative damping and rebalance proportional/integral action.
  const PidGains tuned{9.0, 7.5, 0.65};
  const auto baseline_data = simulate(baseline, 1.0);
  const auto tuned_data = simulate(tuned, 1.0);
  const Metrics baseline_metrics = calculate_metrics(baseline_data, 55.0);
  const Metrics tuned_metrics = calculate_metrics(tuned_data, 55.0);
  write_step_csv(output_dir / "step_response_baseline.csv", baseline_data);
  write_step_csv(output_dir / "step_response_tuned_nominal.csv", tuned_data);
  for (const double load : {0.80, 1.00, 1.25})
    write_step_csv(output_dir / ("step_response_load_" + std::to_string(static_cast<int>(load * 100)) + ".csv"), simulate(tuned, load));
  write_frequency_data(output_dir, tuned);
  write_root_locus(output_dir / "root_locus.csv", tuned);

  std::ofstream summary(output_dir / "metrics_summary.csv");
  summary << "controller,overshoot_pct,rise_time_s,settling_time_s,steady_state_error_deg\n";
  summary << "baseline," << baseline_metrics.overshoot_pct << ',' << baseline_metrics.rise_time_s << ',' << baseline_metrics.settling_time_s << ',' << baseline_metrics.steady_state_error_deg << '\n';
  summary << "tuned," << tuned_metrics.overshoot_pct << ',' << tuned_metrics.rise_time_s << ',' << tuned_metrics.settling_time_s << ',' << tuned_metrics.steady_state_error_deg << '\n';

  print_metrics("Baseline", baseline_metrics);
  print_metrics("Tuned", tuned_metrics);
  const double overshoot_reduction = baseline_metrics.overshoot_pct > 0 ?
      100.0 * (baseline_metrics.overshoot_pct - tuned_metrics.overshoot_pct) / baseline_metrics.overshoot_pct : 0.0;
  const double settling_reduction = std::isfinite(baseline_metrics.settling_time_s) && std::isfinite(tuned_metrics.settling_time_s) ?
      100.0 * (baseline_metrics.settling_time_s - tuned_metrics.settling_time_s) / baseline_metrics.settling_time_s : 0.0;
  std::cout << "Improvement: overshoot reduced " << overshoot_reduction << "% | settling time reduced " << settling_reduction << "%\n";
  std::cout << "Wrote CSV analyses to " << output_dir.string() << '\n';
  return 0;
}
