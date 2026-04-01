#include "tmc2209.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <future>
#include <limits>
#include <stdexcept>

namespace tmcstepper {

namespace {

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

std::string optional_string_attr(const viam::sdk::ProtoStruct& attrs, const std::string& key,
                                 const std::string& default_val) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        return default_val;
    }
    const auto* val = it->second.get<std::string>();
    return val ? *val : default_val;
}

void timespec_add_ns(struct timespec& ts, uint64_t ns) {
    ts.tv_nsec += static_cast<long>(ns);
    while (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000L;
    }
}

}  // namespace

std::vector<std::string> Tmc2209::validate(const viam::sdk::ResourceConfig& cfg) {
    auto attrs = cfg.attributes();

    require_number_attr(attrs, "step_pin");
    require_number_attr(attrs, "dir_pin");

    auto spr = require_number_attr(attrs, "steps_per_revolution");
    if (spr <= 0) {
        throw std::invalid_argument("steps_per_revolution must be positive");
    }

    return {};
}

Tmc2209::Tmc2209(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg)
    : Motor(cfg.name()), has_enn_(false), steps_per_revolution_(0), max_rpm_(0) {
    auto attrs = cfg.attributes();

    auto chip_name = optional_string_attr(attrs, "gpio_chip", "gpiochip0");
    step_offset_ = static_cast<unsigned int>(require_number_attr(attrs, "step_pin"));
    dir_offset_ = static_cast<unsigned int>(require_number_attr(attrs, "dir_pin"));

    auto enn_it = attrs.find("enn_pin");
    if (enn_it != attrs.end()) {
        const auto* val = enn_it->second.get<double>();
        if (val) {
            enn_offset_ = static_cast<unsigned int>(*val);
            has_enn_ = true;
        }
    }

    steps_per_revolution_ = static_cast<int>(require_number_attr(attrs, "steps_per_revolution"));
    max_rpm_ = optional_number_attr(attrs, "max_rpm", 0);

    ::gpiod::chip chip(chip_name);

    ::gpiod::line_settings output_settings;
    output_settings.set_direction(::gpiod::line::direction::OUTPUT);
    output_settings.set_output_value(::gpiod::line::value::INACTIVE);

    auto builder = chip.prepare_request();
    builder.set_consumer("tmcstepper-tmc2209");
    builder.add_line_settings(step_offset_, output_settings);
    builder.add_line_settings(dir_offset_, output_settings);

    if (has_enn_) {
        ::gpiod::line_settings enn_settings;
        enn_settings.set_direction(::gpiod::line::direction::OUTPUT);
        enn_settings.set_output_value(::gpiod::line::value::ACTIVE);
        builder.add_line_settings(enn_offset_, enn_settings);
    }

    gpio_request_.emplace(builder.do_request());

    if (has_enn_) {
        gpio_request_->set_value(enn_offset_, ::gpiod::line::value::ACTIVE);
    }
    gpio_request_->set_value(step_offset_, ::gpiod::line::value::INACTIVE);

    step_position_.store(0);
    target_step_position_.store(0);
}

Tmc2209::~Tmc2209() {
    cancel_stepping();
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

void Tmc2209::start_stepping(bool forward, uint64_t freq_hz) {
    gpio_request_->set_value(dir_offset_,
                            forward ? ::gpiod::line::value::ACTIVE
                                    : ::gpiod::line::value::INACTIVE);
    if (has_enn_) {
        gpio_request_->set_value(enn_offset_, ::gpiod::line::value::INACTIVE);
    }

    cancel_flag_.store(false);
    stepping_thread_ = std::thread([this, freq_hz]() {
        int64_t target = target_step_position_.load();
        bool fwd = gpio_request_->get_value(dir_offset_) == ::gpiod::line::value::ACTIVE;
        step_loop(target, fwd, freq_hz);
    });
}

void Tmc2209::stop_hardware() {
    gpio_request_->set_value(step_offset_, ::gpiod::line::value::INACTIVE);
    if (has_enn_) {
        gpio_request_->set_value(enn_offset_, ::gpiod::line::value::ACTIVE);
    }
}

void Tmc2209::cancel_stepping() {
    cancel_flag_.store(true);
    if (stepping_thread_.joinable()) {
        stepping_thread_.join();
    }
    cancel_flag_.store(false);
}

void Tmc2209::step_loop(int64_t target, bool forward, uint64_t freq_hz) {
    bool indefinite =
        (target == std::numeric_limits<int64_t>::max() ||
         target == std::numeric_limits<int64_t>::min());

    uint64_t half_period_ns = 500000000ULL / freq_hz;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!cancel_flag_.load(std::memory_order_relaxed)) {
        gpio_request_->set_value(step_offset_, ::gpiod::line::value::ACTIVE);
        timespec_add_ns(next, half_period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

        gpio_request_->set_value(step_offset_, ::gpiod::line::value::INACTIVE);
        timespec_add_ns(next, half_period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

        int64_t pos = step_position_.load(std::memory_order_relaxed);
        pos += forward ? 1 : -1;
        step_position_.store(pos, std::memory_order_relaxed);

        if (!indefinite) {
            if ((forward && pos >= target) || (!forward && pos <= target)) {
                step_position_.store(target, std::memory_order_relaxed);
                try {
                    stop_hardware();
                } catch (...) {
                }
                return;
            }
        }
    }

    gpio_request_->set_value(step_offset_, ::gpiod::line::value::INACTIVE);
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
    cancel_stepping();

    bool forward = power_pct > 0;
    double freq = std::abs(power_pct) * max_rpm_ * static_cast<double>(steps_per_revolution_) / 60.0;
    auto freq_hz = std::max(uint64_t{1}, static_cast<uint64_t>(std::round(freq)));

    int64_t target = forward ? std::numeric_limits<int64_t>::max()
                             : std::numeric_limits<int64_t>::min();
    target_step_position_.store(target);

    start_stepping(forward, freq_hz);
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

    auto promise = std::make_shared<std::promise<void>>();
    std::future<void> done_future = promise->get_future();

    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_stepping();

        gpio_request_->set_value(dir_offset_,
                                forward ? ::gpiod::line::value::ACTIVE
                                        : ::gpiod::line::value::INACTIVE);
        if (has_enn_) {
            gpio_request_->set_value(enn_offset_, ::gpiod::line::value::INACTIVE);
        }

        cancel_flag_.store(false);
        stepping_thread_ = std::thread([this, target, forward, freq_hz, promise]() {
            step_loop(target, forward, freq_hz);
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
    cancel_stepping();

    bool forward = rpm > 0;
    int64_t target = forward ? std::numeric_limits<int64_t>::max()
                             : std::numeric_limits<int64_t>::min();
    target_step_position_.store(target);

    auto freq_hz = rpm_to_freq_hz(rpm);
    start_stepping(forward, freq_hz);
}

void Tmc2209::stop(const viam::sdk::ProtoStruct& extra) {
    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_stepping();
    }
    target_step_position_.store(step_position_.load());
    stop_hardware();
}

void Tmc2209::reset_zero_position(double offset, const viam::sdk::ProtoStruct& extra) {
    {
        std::lock_guard<std::mutex> guard(lock_);
        cancel_stepping();
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
