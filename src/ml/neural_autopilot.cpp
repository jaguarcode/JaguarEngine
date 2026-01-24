/**
 * @file neural_autopilot.cpp
 * @brief Implementation of neural network-based autopilot
 */

#include "jaguar/ml/neural_autopilot.h"
#include "jaguar/ml/inference_session.h"
#include <cmath>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <stdexcept>

namespace jaguar::ml {

// ============================================================================
// Quaternion Implementation
// ============================================================================

void Quaternion::to_euler(Real& roll, Real& pitch, Real& yaw) const noexcept {
    // Convert quaternion to Euler angles (ZYX convention)
    // Roll (x-axis rotation)
    Real sinr_cosp = 2.0 * (w * x + y * z);
    Real cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    Real sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(constants::PI / 2.0, sinp); // Use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    Real siny_cosp = 2.0 * (w * z + x * y);
    Real cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Quaternion Quaternion::from_euler(Real roll, Real pitch, Real yaw) noexcept {
    // Convert Euler angles to quaternion (ZYX convention)
    Real cr = std::cos(roll * 0.5);
    Real sr = std::sin(roll * 0.5);
    Real cp = std::cos(pitch * 0.5);
    Real sp = std::sin(pitch * 0.5);
    Real cy = std::cos(yaw * 0.5);
    Real sy = std::sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// ============================================================================
// DefaultObservationNormalizer
// ============================================================================

class DefaultObservationNormalizer : public IObservationNormalizer {
public:
    explicit DefaultObservationNormalizer(VehicleType vehicle_type)
        : vehicle_type_(vehicle_type) {}

    std::vector<Real> normalize(const AutopilotObservation& obs) override {
        std::vector<Real> normalized;
        normalized.reserve(32); // Pre-allocate expected size

        // Normalize position (assume scenario bounds -10000 to 10000 meters)
        constexpr Real pos_scale = 10000.0;
        normalized.push_back(clamp(obs.position.x / pos_scale, -1.0, 1.0));
        normalized.push_back(clamp(obs.position.y / pos_scale, -1.0, 1.0));
        normalized.push_back(clamp(obs.position.z / pos_scale, -1.0, 1.0));

        // Normalize velocity (scale by max expected speed)
        Real max_speed = (vehicle_type_ == VehicleType::Aircraft) ? 300.0 : 30.0;
        normalized.push_back(clamp(obs.velocity.x / max_speed, -1.0, 1.0));
        normalized.push_back(clamp(obs.velocity.y / max_speed, -1.0, 1.0));
        normalized.push_back(clamp(obs.velocity.z / max_speed, -1.0, 1.0));

        // Normalize orientation to Euler angles
        Real roll, pitch, yaw;
        obs.orientation.to_euler(roll, pitch, yaw);
        normalized.push_back(roll / constants::PI);     // [-1, 1]
        normalized.push_back(pitch / constants::PI);    // [-1, 1]
        normalized.push_back(yaw / constants::PI);      // [-1, 1]

        // Normalize angular velocity (deg/s to normalized)
        constexpr Real ang_vel_scale = 60.0 * constants::DEG_TO_RAD; // 60 deg/s
        normalized.push_back(clamp(obs.angular_velocity.x / ang_vel_scale, -1.0, 1.0));
        normalized.push_back(clamp(obs.angular_velocity.y / ang_vel_scale, -1.0, 1.0));
        normalized.push_back(clamp(obs.angular_velocity.z / ang_vel_scale, -1.0, 1.0));

        // Normalize altitude
        Real max_altitude = (vehicle_type_ == VehicleType::Aircraft) ? 15000.0 : 100.0;
        normalized.push_back(clamp(obs.altitude / max_altitude, -0.1, 1.0));

        // Normalize airspeed
        normalized.push_back(clamp(obs.airspeed / max_speed, 0.0, 1.0));

        // Normalize heading, pitch, roll (already in radians)
        normalized.push_back(obs.heading / constants::PI);
        normalized.push_back(obs.pitch / constants::PI);
        normalized.push_back(obs.roll / constants::PI);

        // Normalize target position (relative to current position)
        Vec3 target_delta = obs.target_position - obs.position;
        normalized.push_back(clamp(target_delta.x / pos_scale, -1.0, 1.0));
        normalized.push_back(clamp(target_delta.y / pos_scale, -1.0, 1.0));
        normalized.push_back(clamp(target_delta.z / pos_scale, -1.0, 1.0));

        // Normalize target altitude (relative)
        Real altitude_delta = obs.target_altitude - obs.altitude;
        normalized.push_back(clamp(altitude_delta / 1000.0, -1.0, 1.0));

        // Normalize target speed
        normalized.push_back(clamp(obs.target_speed / max_speed, 0.0, 1.0));

        // Normalize target heading (relative)
        Real heading_delta = heading_error(obs.heading, obs.target_heading);
        normalized.push_back(heading_delta / constants::PI);

        return normalized;
    }

    AutopilotObservation denormalize(const std::vector<Real>& normalized) override {
        // Not typically needed for inference, but implemented for completeness
        AutopilotObservation obs;
        if (normalized.size() < 24) return obs;

        constexpr Real pos_scale = 10000.0;
        Real max_speed = (vehicle_type_ == VehicleType::Aircraft) ? 300.0 : 30.0;

        obs.position.x = normalized[0] * pos_scale;
        obs.position.y = normalized[1] * pos_scale;
        obs.position.z = normalized[2] * pos_scale;

        obs.velocity.x = normalized[3] * max_speed;
        obs.velocity.y = normalized[4] * max_speed;
        obs.velocity.z = normalized[5] * max_speed;

        Real roll = normalized[6] * constants::PI;
        Real pitch = normalized[7] * constants::PI;
        Real yaw = normalized[8] * constants::PI;
        obs.orientation = Quaternion::from_euler(roll, pitch, yaw);

        obs.altitude = normalized[12] * (vehicle_type_ == VehicleType::Aircraft ? 15000.0 : 100.0);
        obs.airspeed = normalized[13] * max_speed;
        obs.heading = normalized[14] * constants::PI;
        obs.pitch = normalized[15] * constants::PI;
        obs.roll = normalized[16] * constants::PI;

        return obs;
    }

private:
    VehicleType vehicle_type_;
};

// ============================================================================
// DefaultActionDenormalizer
// ============================================================================

class DefaultActionDenormalizer : public IActionDenormalizer {
public:
    AutopilotAction denormalize(const std::vector<Real>& normalized) override {
        AutopilotAction action;
        if (normalized.size() < 7) return action;

        // Neural network outputs are expected to be in [-1, 1] range
        action.elevator = clamp(normalized[0], -1.0, 1.0);
        action.aileron = clamp(normalized[1], -1.0, 1.0);
        action.rudder = clamp(normalized[2], -1.0, 1.0);
        action.throttle = clamp((normalized[3] + 1.0) * 0.5, 0.0, 1.0); // [-1,1] to [0,1]
        action.collective = clamp((normalized[4] + 1.0) * 0.5, 0.0, 1.0);
        action.flaps = clamp((normalized[5] + 1.0) * 0.5, 0.0, 1.0);
        action.speedbrake = clamp((normalized[6] + 1.0) * 0.5, 0.0, 1.0);

        return action;
    }

    std::vector<Real> normalize(const AutopilotAction& action) override {
        std::vector<Real> normalized;
        normalized.reserve(7);

        normalized.push_back(action.elevator);
        normalized.push_back(action.aileron);
        normalized.push_back(action.rudder);
        normalized.push_back(action.throttle * 2.0 - 1.0); // [0,1] to [-1,1]
        normalized.push_back(action.collective * 2.0 - 1.0);
        normalized.push_back(action.flaps * 2.0 - 1.0);
        normalized.push_back(action.speedbrake * 2.0 - 1.0);

        return normalized;
    }
};

// ============================================================================
// DefaultSafetyMonitor
// ============================================================================

class DefaultSafetyMonitor : public ISafetyMonitor {
public:
    AutopilotAction apply_constraints(
        const AutopilotAction& action,
        const AutopilotObservation& observation,
        const SafetyConstraints& constraints) override {

        AutopilotAction safe_action = action;

        // Clamp control surface ranges
        safe_action.elevator = clamp(action.elevator, -1.0, 1.0);
        safe_action.aileron = clamp(action.aileron, -1.0, 1.0);
        safe_action.rudder = clamp(action.rudder, -1.0, 1.0);
        safe_action.throttle = clamp(action.throttle, 0.0, 1.0);
        safe_action.collective = clamp(action.collective, 0.0, 1.0);
        safe_action.flaps = clamp(action.flaps, 0.0, 1.0);
        safe_action.speedbrake = clamp(action.speedbrake, 0.0, 1.0);

        // Apply rate limits (approximate with small time step)
        constexpr Real dt = 0.02; // 50Hz control rate
        (void)dt; // Rate limiting simplified for stub implementation

        // Check altitude constraints
        if (observation.altitude < constraints.min_altitude) {
            // Force climb
            safe_action.elevator = std::max(safe_action.elevator, 0.1);
            safe_action.throttle = std::max(safe_action.throttle, 0.7);
        } else if (observation.altitude > constraints.max_altitude) {
            // Force descent
            safe_action.elevator = std::min(safe_action.elevator, -0.1);
            safe_action.throttle = std::min(safe_action.throttle, 0.3);
        }

        // Check speed constraints
        if (observation.airspeed < constraints.min_speed) {
            // Increase throttle
            safe_action.throttle = std::max(safe_action.throttle, 0.8);
        } else if (observation.airspeed > constraints.max_speed) {
            // Reduce throttle
            safe_action.throttle = std::min(safe_action.throttle, 0.2);
            safe_action.speedbrake = std::max(safe_action.speedbrake, 0.5);
        }

        // Check attitude constraints
        Real roll_deg = std::abs(observation.roll * constants::RAD_TO_DEG);
        Real pitch_deg = std::abs(observation.pitch * constants::RAD_TO_DEG);

        if (roll_deg > constraints.max_bank_angle) {
            // Reduce bank
            safe_action.aileron = -std::copysign(0.5, observation.roll);
        }

        if (pitch_deg > constraints.max_pitch_angle) {
            // Reduce pitch
            safe_action.elevator = -std::copysign(0.3, observation.pitch);
        }

        return safe_action;
    }

    bool is_safe(
        const AutopilotAction& action,
        const AutopilotObservation& observation,
        const SafetyConstraints& constraints) override {

        // Check if action would violate constraints
        if (observation.altitude < constraints.min_altitude && action.elevator < 0.0) {
            return false;
        }

        if (observation.altitude > constraints.max_altitude && action.elevator > 0.0) {
            return false;
        }

        if (observation.airspeed < constraints.min_speed && action.throttle < 0.5) {
            return false;
        }

        if (observation.airspeed > constraints.max_speed && action.throttle > 0.5) {
            return false;
        }

        Real roll_deg = std::abs(observation.roll * constants::RAD_TO_DEG);
        Real pitch_deg = std::abs(observation.pitch * constants::RAD_TO_DEG);

        if (roll_deg > constraints.max_bank_angle) {
            return false;
        }

        if (pitch_deg > constraints.max_pitch_angle) {
            return false;
        }

        return true;
    }
};

// ============================================================================
// NeuralAutopilotImpl::Impl
// ============================================================================

struct NeuralAutopilotImplData {
    AutopilotConfig config;
    AutopilotMode mode{AutopilotMode::Manual};

    std::unique_ptr<InferenceSession> inference_session;
    std::unique_ptr<IObservationNormalizer> normalizer;
    std::unique_ptr<IActionDenormalizer> denormalizer;
    std::unique_ptr<ISafetyMonitor> safety_monitor;

    Vec3 target_position{0.0, 0.0, 0.0};
    Real target_altitude{0.0};
    Real target_speed{0.0};
    Real target_heading{0.0};

    std::atomic<bool> initialized{false};
    mutable std::mutex mutex;

    AutopilotStats stats;
    AutopilotAction previous_action;

    NeuralAutopilotImplData(const AutopilotConfig& cfg) : config(cfg) {}

    // PID controller state for fallback
    struct PIDState {
        Real altitude_error_integral{0.0};
        Real heading_error_integral{0.0};
        Real speed_error_integral{0.0};
        Real prev_altitude_error{0.0};
        Real prev_heading_error{0.0};
        Real prev_speed_error{0.0};
    } pid_state;

    AutopilotAction compute_pid_fallback(const AutopilotObservation& obs) {
        // Simple PID controller for basic autopilot when no neural model
        AutopilotAction action = AutopilotAction::neutral();

        constexpr Real dt = 0.02; // 50Hz update rate

        // PID gains (basic tuning)
        constexpr Real altitude_kp = 0.05;
        constexpr Real altitude_ki = 0.001;
        constexpr Real altitude_kd = 0.1;

        constexpr Real heading_kp = 0.3;
        constexpr Real heading_ki = 0.01;
        constexpr Real heading_kd = 0.2;

        constexpr Real speed_kp = 0.02;
        constexpr Real speed_ki = 0.001;
        constexpr Real speed_kd = 0.01;

        // Altitude control (elevator)
        if (mode == AutopilotMode::Altitude || mode == AutopilotMode::Waypoint) {
            Real alt_error = altitude_error(obs.altitude, target_altitude);
            pid_state.altitude_error_integral += alt_error * dt;
            pid_state.altitude_error_integral = clamp(pid_state.altitude_error_integral, -100.0, 100.0);
            Real alt_derivative = (alt_error - pid_state.prev_altitude_error) / dt;

            action.elevator = altitude_kp * alt_error +
                            altitude_ki * pid_state.altitude_error_integral +
                            altitude_kd * alt_derivative;
            action.elevator = clamp(action.elevator, -1.0, 1.0);

            pid_state.prev_altitude_error = alt_error;
        }

        // Heading control (rudder/aileron)
        if (mode == AutopilotMode::Course || mode == AutopilotMode::Waypoint) {
            Real heading_err = heading_error(obs.heading, target_heading);
            pid_state.heading_error_integral += heading_err * dt;
            pid_state.heading_error_integral = clamp(pid_state.heading_error_integral, -10.0, 10.0);
            Real heading_derivative = (heading_err - pid_state.prev_heading_error) / dt;

            Real heading_correction = heading_kp * heading_err +
                                    heading_ki * pid_state.heading_error_integral +
                                    heading_kd * heading_derivative;

            // Use coordinated turn (aileron + rudder)
            action.aileron = clamp(heading_correction, -1.0, 1.0);
            action.rudder = clamp(heading_correction * 0.3, -1.0, 1.0);

            pid_state.prev_heading_error = heading_err;
        }

        // Speed control (throttle)
        if (mode == AutopilotMode::Speed || mode == AutopilotMode::Waypoint) {
            Real speed_err = speed_error(obs.airspeed, target_speed);
            pid_state.speed_error_integral += speed_err * dt;
            pid_state.speed_error_integral = clamp(pid_state.speed_error_integral, -50.0, 50.0);
            Real speed_derivative = (speed_err - pid_state.prev_speed_error) / dt;

            Real throttle_correction = speed_kp * speed_err +
                                     speed_ki * pid_state.speed_error_integral +
                                     speed_kd * speed_derivative;

            // Baseline throttle for level flight
            action.throttle = clamp(0.5 + throttle_correction, 0.0, 1.0);

            pid_state.prev_speed_error = speed_err;
        } else {
            action.throttle = 0.5; // Default cruise throttle
        }

        // Waypoint mode: compute heading to target
        if (mode == AutopilotMode::Waypoint) {
            target_heading = bearing_to_target(obs.position, target_position);
        }

        return action;
    }
};

// ============================================================================
// NeuralAutopilot Implementation
// ============================================================================

class NeuralAutopilotImpl : public NeuralAutopilot {
public:
    explicit NeuralAutopilotImpl(const AutopilotConfig& config)
        : NeuralAutopilot(config)
        , impl_(std::make_unique<NeuralAutopilotImplData>(config)) {}

    ~NeuralAutopilotImpl() override = default;

    AutopilotResult initialize() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (impl_->initialized.load()) {
            return AutopilotResult::AlreadyInitialized;
        }

        // Create default components
        impl_->normalizer = create_default_normalizer(impl_->config.vehicle_type);
        impl_->denormalizer = create_default_denormalizer(impl_->config.vehicle_type);
        impl_->safety_monitor = create_default_safety_monitor();

        // Initialize inference session if model path provided
        if (!impl_->config.model_path.empty()) {
            InferenceConfig inf_config = InferenceConfig::default_config();
            impl_->inference_session = std::make_unique<InferenceSession>(inf_config);

            auto result = impl_->inference_session->initialize();
            if (result != InferenceResult::Success) {
                // Continue without inference - will use PID fallback
                impl_->inference_session.reset();
            } else {
                // Try to load model
                result = impl_->inference_session->load_model(impl_->config.model_path);
                if (result != InferenceResult::Success) {
                    impl_->inference_session.reset();
                }
            }
        }

        impl_->initialized.store(true);
        impl_->stats.last_update = std::chrono::system_clock::now();

        return AutopilotResult::Success;
    }

    AutopilotResult shutdown() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        if (impl_->inference_session) {
            impl_->inference_session->shutdown();
            impl_->inference_session.reset();
        }

        impl_->initialized.store(false);

        return AutopilotResult::Success;
    }

    bool is_initialized() const override {
        return impl_->initialized.load();
    }

    AutopilotResult load_model(const std::string& path) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->config.model_path = path;

        if (!impl_->inference_session) {
            InferenceConfig inf_config = InferenceConfig::default_config();
            impl_->inference_session = std::make_unique<InferenceSession>(inf_config);

            auto result = impl_->inference_session->initialize();
            if (result != InferenceResult::Success) {
                impl_->inference_session.reset();
                return AutopilotResult::InvalidModel;
            }
        }

        auto result = impl_->inference_session->load_model(path);
        if (result != InferenceResult::Success) {
            return AutopilotResult::InvalidModel;
        }

        return AutopilotResult::Success;
    }

    AutopilotResult set_mode(AutopilotMode mode) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->mode = mode;

        // Reset PID state when changing modes
        impl_->pid_state = {};

        return AutopilotResult::Success;
    }

    AutopilotMode get_mode() const override {
        return impl_->mode;
    }

    AutopilotResult compute(const AutopilotObservation& obs, AutopilotAction& action) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        // Validate observation
        if (!validate_observation(obs)) {
            return AutopilotResult::InvalidObservation;
        }

        // Manual mode - no autopilot control
        if (impl_->mode == AutopilotMode::Manual) {
            action = AutopilotAction::neutral();
            return AutopilotResult::Success;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        AutopilotAction computed_action;

        // Try neural network inference first
        if (impl_->inference_session && impl_->inference_session->is_model_loaded()) {
            try {
                // Normalize observation
                std::vector<Real> normalized_obs = impl_->normalizer->normalize(obs);

                // Get model input info
                auto input_info = impl_->inference_session->get_input_info();
                if (input_info.empty()) {
                    // Fall back to PID
                    computed_action = impl_->compute_pid_fallback(obs);
                } else {
                    // Create input tensor
                    TensorShape input_shape({1, static_cast<Int64>(normalized_obs.size())});
                    std::vector<Tensor> inputs;

                    Tensor input_tensor(input_info[0].name, DataType::Float32, input_shape);
                    std::vector<float> float_obs(normalized_obs.begin(), normalized_obs.end());
                    input_tensor.set_data(float_obs);
                    inputs.push_back(std::move(input_tensor));

                    // Run inference
                    std::vector<Tensor> outputs;
                    auto inf_result = impl_->inference_session->run(inputs, outputs);

                    if (inf_result == InferenceResult::Success && !outputs.empty()) {
                        // Extract output
                        const float* output_data = outputs[0].data_as<float>();
                        if (output_data) {
                            Int64 output_size = outputs[0].element_count();
                            std::vector<Real> normalized_action(output_data, output_data + output_size);

                            // Denormalize action
                            computed_action = impl_->denormalizer->denormalize(normalized_action);
                        } else {
                            computed_action = impl_->compute_pid_fallback(obs);
                        }
                    } else {
                        // Inference failed, use PID fallback
                        computed_action = impl_->compute_pid_fallback(obs);
                    }
                }
            } catch (...) {
                // Exception during inference, fall back to PID
                computed_action = impl_->compute_pid_fallback(obs);
            }
        } else {
            // No model loaded, use PID fallback
            computed_action = impl_->compute_pid_fallback(obs);
        }

        // Apply safety constraints
        if (impl_->config.enable_safety_limits) {
            AutopilotAction safe_action = impl_->safety_monitor->apply_constraints(
                computed_action, obs, impl_->config.safety);

            if (!impl_->safety_monitor->is_safe(safe_action, obs, impl_->config.safety)) {
                impl_->stats.safety_interventions++;
            }

            computed_action = safe_action;
        }

        // Final validation
        if (!validate_action(computed_action)) {
            return AutopilotResult::ComputationError;
        }

        action = clamp_action(computed_action);
        impl_->previous_action = action;

        // Update statistics
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        impl_->stats.total_steps++;
        Real duration_ms = duration.count() / 1000.0;
        impl_->stats.average_inference_ms =
            (impl_->stats.average_inference_ms * (impl_->stats.total_steps - 1) + duration_ms) /
            impl_->stats.total_steps;
        impl_->stats.last_update = std::chrono::system_clock::now();

        return AutopilotResult::Success;
    }

    AutopilotResult set_target(const Vec3& position) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->target_position = position;
        impl_->target_altitude = position.z;

        return AutopilotResult::Success;
    }

    AutopilotResult set_target_altitude(Real altitude) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->target_altitude = altitude;

        return AutopilotResult::Success;
    }

    AutopilotResult set_target_speed(Real speed) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->target_speed = speed;

        return AutopilotResult::Success;
    }

    AutopilotResult set_target_heading(Real heading) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->target_heading = heading;

        return AutopilotResult::Success;
    }

    AutopilotResult set_safety_constraints(const SafetyConstraints& constraints) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return AutopilotResult::NotInitialized;
        }

        impl_->config.safety = constraints;

        return AutopilotResult::Success;
    }

    SafetyConstraints get_safety_constraints() const override {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->config.safety;
    }

    AutopilotStats get_stats() const override {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->stats;
    }

    const AutopilotConfig& get_config() const override {
        return impl_->config;
    }

private:
    std::unique_ptr<NeuralAutopilotImplData> impl_;
};

// ============================================================================
// NeuralAutopilot Base Constructor
// ============================================================================

NeuralAutopilot::NeuralAutopilot(const AutopilotConfig& config) {
    (void)config;  // Base class constructor - impl is in derived class
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<NeuralAutopilot> create_neural_autopilot(const AutopilotConfig& config) {
    return std::make_unique<NeuralAutopilotImpl>(config);
}

std::unique_ptr<NeuralAutopilot> create_aircraft_autopilot(const std::string& model_path) {
    return create_neural_autopilot(AutopilotConfig::aircraft(model_path));
}

std::unique_ptr<NeuralAutopilot> create_ship_autopilot(const std::string& model_path) {
    return create_neural_autopilot(AutopilotConfig::ship(model_path));
}

std::unique_ptr<IObservationNormalizer> create_default_normalizer(VehicleType vehicle_type) {
    return std::make_unique<DefaultObservationNormalizer>(vehicle_type);
}

std::unique_ptr<IActionDenormalizer> create_default_denormalizer(VehicleType vehicle_type) {
    (void)vehicle_type; // Unused for now - same denormalizer for all types
    return std::make_unique<DefaultActionDenormalizer>();
}

std::unique_ptr<ISafetyMonitor> create_default_safety_monitor() {
    return std::make_unique<DefaultSafetyMonitor>();
}

} // namespace jaguar::ml
