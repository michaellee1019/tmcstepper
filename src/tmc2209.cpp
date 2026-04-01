#include "tmc2209.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include <viam/sdk/components/board.hpp>
#include <viam/sdk/log/logging.hpp>

namespace tmcstepper {

namespace {

std::string require_string_attr(const viam::sdk::ProtoStruct& attrs, const std::string& key) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        throw std::invalid_argument("missing required attribute: " + key);
    }
    const auto* val = it->second.get<std::string>();
    if (!val || val->empty()) {
        throw std::invalid_argument("attribute '" + key + "' must be a non-empty string");
    }
    return *val;
}

std::string optional_string_attr(const viam::sdk::ProtoStruct& attrs, const std::string& key) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        return "";
    }
    const auto* val = it->second.get<std::string>();
    return val ? *val : "";
}

double require_number_attr(const viam::sdk::ProtoStruct& attrs, const std::string& key) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        throw std::invalid_argument("missing required attribute: " + key);
    }
    const auto* val = it->second.get<double>();
    if (!val) {
        throw std::invalid_argument("attribute '" + key + "' must be a number");
    }
    return *val;
}

double optional_number_attr(const viam::sdk::ProtoStruct& attrs, const std::string& key,
                            double default_val) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        return default_val;
    }
    const auto* val = it->second.get<double>();
    return val ? *val : default_val;
}

}  // namespace

std::vector<std::string> Tmc2209::validate(const viam::sdk::ResourceConfig& cfg) {
    auto attrs = cfg.attributes();

    auto board_name = require_string_attr(attrs, "board");

    auto pins_it = attrs.find("pins");
    if (pins_it == attrs.end()) {
        throw std::invalid_argument("missing required attribute: pins");
    }
    const auto* pins = pins_it->second.get<viam::sdk::ProtoStruct>();
    if (!pins) {
        throw std::invalid_argument("attribute 'pins' must be a struct");
    }
    require_string_attr(*pins, "step");
    require_string_attr(*pins, "dir");

    auto spr = require_number_attr(attrs, "steps_per_revolution");
    if (spr <= 0) {
        throw std::invalid_argument("steps_per_revolution must be positive");
    }

    return {board_name};
}

Tmc2209::Tmc2209(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg)
    : Motor(cfg.name()), max_rpm_(0) {
    auto attrs = cfg.attributes();

    auto board_name = require_string_attr(attrs, "board");
    for (const auto& dep : deps) {
        if (dep.first.short_name() == board_name) {
            board_ = std::dynamic_pointer_cast<viam::sdk::Board>(dep.second);
            break;
        }
    }
    if (!board_) {
        throw std::invalid_argument("board dependency '" + board_name + "' not found or wrong type");
    }

    const auto* pins = attrs.at("pins").get<viam::sdk::ProtoStruct>();
    step_pin_ = require_string_attr(*pins, "step");
    dir_pin_ = require_string_attr(*pins, "dir");
    enn_pin_ = optional_string_attr(*pins, "enn");

    steps_per_revolution_ = static_cast<int>(require_number_attr(attrs, "steps_per_revolution"));
    max_rpm_ = optional_number_attr(attrs, "max_rpm", 0);

    // ENN is active low on TMC2209: set HIGH to disable the driver on startup
    if (!enn_pin_.empty()) {
        board_->set_gpio(enn_pin_, true);
    }

    step_position_.store(0);
    target_step_position_.store(0);
}

Tmc2209::~Tmc2209() {
    cancel_tracking();
    try {
        stop_hardware();
    } catch (...) {
    }
}

uint64_t Tmc2209::rpm_to_freq_hz(double rpm) const {
    double freq = std::abs(rpm) * static_cast<double>(steps_per_revolution_) / 60.0;
    if (max_rpm_ > 0) {
        double max_freq = max_rpm_ * static_cast<double>(steps_per_revolution_) / 60.0;
        freq = std::min(freq, max_freq);
    }
    return std::max(uint64_t{1}, static_cast<uint64_t>(std::round(freq)));
}

void Tmc2209::start_pwm(bool forward, uint64_t freq_hz) {
    board_->set_gpio(dir_pin_, forward);
    if (!enn_pin_.empty()) {
        board_->set_gpio(enn_pin_, false);  // active low: LOW enables
    }
    board_->set_pwm_frequency(step_pin_, freq_hz);
    board_->set_pwm_duty_cycle(step_pin_, 0.5);
}

void Tmc2209::stop_hardware() {
    board_->set_pwm_duty_cycle(step_pin_, 0.0);
    if (!enn_pin_.empty()) {
        board_->set_gpio(enn_pin_, true);  // active low: HIGH disables
    }
}

void Tmc2209::cancel_tracking() {
    cancel_flag_.store(true);
    if (tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
    cancel_flag_.store(false);
}

void Tmc2209::track_position(int64_t target, bool forward, double freq_hz) {
    using clock = std::chrono::steady_clock;
    using fsec = std::chrono::duration<double>;

    auto last = clock::now();
    double accumulator = 0.0;
    bool indefinite =
        (target == std::numeric_limits<int64_t>::max() ||
         target == std::numeric_limits<int64_t>::min());

    while (!cancel_flag_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto now = clock::now();
        double elapsed = std::chrono::duration_cast<fsec>(now - last).count();
        last = now;

        accumulator += elapsed * freq_hz;
        auto whole_steps = static_cast<int64_t>(accumulator);
        accumulator -= static_cast<double>(whole_steps);

        int64_t cur = step_position_.load();
        if (forward) {
            cur += whole_steps;
        } else {
            cur -= whole_steps;
        }

        if (!indefinite) {
            if ((forward && cur >= target) || (!forward && cur <= target)) {
                step_position_.store(target);
                try {
                    stop_hardware();
                } catch (...) {
                }
                return;
            }
        }
        step_position_.store(cur);
    }
}

void Tmc2209::set_power(double power_pct, const viam::sdk::ProtoStruct& extra) {
    if (std::abs(power_pct) < 0.0001) {
        stop(extra);
        return;
    }

    if (max_rpm_ <= 0) {
        throw std::runtime_error(
            "set_power requires 'max_rpm' to be configured to determine step frequency");
    }

    std::lock_guard<std::mutex> guard(lock_);
    cancel_tracking();

    bool forward = power_pct > 0;
    double freq = std::abs(power_pct) * max_rpm_ * static_cast<double>(steps_per_revolution_) / 60.0;
    auto freq_hz = std::max(uint64_t{1}, static_cast<uint64_t>(std::round(freq)));

    int64_t target = forward ? std::numeric_limits<int64_t>::max()
                             : std::numeric_limits<int64_t>::min();
    target_step_position_.store(target);

    start_pwm(forward, freq_hz);

    cancel_flag_.store(false);
    tracking_thread_ = std::thread([this, target, forward, freq_hz]() {
        track_position(target, forward, static_cast<double>(freq_hz));
    });
}

void Tmc2209::go_for(double rpm, double revolutions, const viam::sdk::ProtoStruct& extra) {
    if (std::abs(rpm) < 0.1) {
        stop(extra);
        throw std::runtime_error("RPM too low (near zero)");
    }

    if (revolutions == 0) {
        set_rpm(rpm, extra);
        return;
    }

    int64_t direction = 1;
    if (std::signbit(revolutions) != std::signbit(rpm)) {
        direction = -1;
    }
    bool forward = direction > 0;

    int64_t steps =
        static_cast<int64_t>(std::abs(revolutions) * static_cast<double>(steps_per_revolution_));
    int64_t target = step_position_.load() + direction * steps;
    target_step_position_.store(target);

    auto freq_hz = rpm_to_freq_hz(rpm);

    // Use a promise so go_for can block without holding the lock. This avoids a
    // race where stop() tries to join the same thread we're waiting on.
    auto promise = std::make_shared<std::promise<void>>();
    std::future<void> done_future = promise->get_future();

    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_tracking();
        start_pwm(forward, freq_hz);

        cancel_flag_.store(false);
        tracking_thread_ = std::thread([this, target, forward, freq_hz, promise]() {
            track_position(target, forward, static_cast<double>(freq_hz));
            promise->set_value();
        });
    }

    done_future.wait();
}

void Tmc2209::go_to(double rpm, double position_revolutions, const viam::sdk::ProtoStruct& extra) {
    double cur_pos = get_position(extra);
    double distance = position_revolutions - cur_pos;

    if (std::abs(distance) < 0.001) {
        return;
    }

    go_for(std::abs(rpm), distance, extra);
}

void Tmc2209::set_rpm(double rpm, const viam::sdk::ProtoStruct& extra) {
    if (std::abs(rpm) < 0.0001) {
        stop(extra);
        return;
    }

    std::lock_guard<std::mutex> guard(lock_);
    cancel_tracking();

    bool forward = rpm > 0;
    int64_t target = forward ? std::numeric_limits<int64_t>::max()
                             : std::numeric_limits<int64_t>::min();
    target_step_position_.store(target);

    auto freq_hz = rpm_to_freq_hz(rpm);
    start_pwm(forward, freq_hz);

    cancel_flag_.store(false);
    tracking_thread_ = std::thread([this, target, forward, freq_hz]() {
        track_position(target, forward, static_cast<double>(freq_hz));
    });
}

void Tmc2209::stop(const viam::sdk::ProtoStruct& extra) {
    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_tracking();
    }
    target_step_position_.store(step_position_.load());
    stop_hardware();
}

void Tmc2209::reset_zero_position(double offset, const viam::sdk::ProtoStruct& extra) {
    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_tracking();
    }
    stop_hardware();
    int64_t pos =
        static_cast<int64_t>(-offset * static_cast<double>(steps_per_revolution_));
    step_position_.store(pos);
    target_step_position_.store(pos);
}

viam::sdk::Motor::position Tmc2209::get_position(const viam::sdk::ProtoStruct& extra) {
    return static_cast<double>(step_position_.load()) /
           static_cast<double>(steps_per_revolution_);
}

viam::sdk::Motor::properties Tmc2209::get_properties(const viam::sdk::ProtoStruct& extra) {
    return {true};
}

viam::sdk::Motor::power_status Tmc2209::get_power_status(const viam::sdk::ProtoStruct& extra) {
    bool on = is_moving();
    double pct = on ? 1.0 : 0.0;
    return {on, pct};
}

bool Tmc2209::is_moving() {
    return step_position_.load() != target_step_position_.load();
}

viam::sdk::ProtoStruct Tmc2209::get_status() {
    return {};
}

viam::sdk::ProtoStruct Tmc2209::do_command(const viam::sdk::ProtoStruct& command) {
    return {};
}

std::vector<viam::sdk::GeometryConfig> Tmc2209::get_geometries(
    const viam::sdk::ProtoStruct& extra) {
    return {};
}

}  // namespace tmcstepper
