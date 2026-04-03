#include "tmc2209.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>

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

std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

unsigned int parse_pin_offset(const std::string& raw, const std::string& key) {
    std::string s = trim_copy(raw);
    if (s.empty()) {
        throw std::invalid_argument("attribute '" + key + "' must be a non-empty pin string");
    }
    char* end = nullptr;
    unsigned long ul = std::strtoul(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') {
        throw std::invalid_argument("attribute '" + key + "' must be a decimal line offset string");
    }
    if (ul > static_cast<unsigned long>(std::numeric_limits<unsigned int>::max())) {
        throw std::invalid_argument("attribute '" + key + "' line offset out of range");
    }
    return static_cast<unsigned int>(ul);
}

unsigned int pin_attr_to_offset(const viam::sdk::ProtoStruct& attrs, const std::string& key) {
    auto it = attrs.find(key);
    if (it == attrs.end()) {
        throw std::invalid_argument("missing required attribute: " + key);
    }
    const auto* str = it->second.get<std::string>();
    if (str) {
        return parse_pin_offset(*str, key);
    }
    const auto* num = it->second.get<double>();
    if (num) {
        if (*num < 0 || *num != std::floor(*num)) {
            throw std::invalid_argument("attribute '" + key + "' must be a non-negative integer");
        }
        return static_cast<unsigned int>(*num);
    }
    throw std::invalid_argument("attribute '" + key + "' must be a string (recommended) or number");
}

std::string normalize_gpio_chip_path(std::string name) {
    if (name.empty()) {
        name = "gpiochip0";
    }
    if (name.front() != '/') {
        return "/dev/" + name;
    }
    return name;
}

// Ordered for Raspberry Pi family: BCM boards first, then Pi 5 (RP1) gpiochips.
::gpiod::chip open_gpio_chip(const std::string& user_chip_trimmed) {
    static constexpr const char* k_pi_auto_candidates[] = {
        "/dev/gpiochip0",   // Pi 3 / 4 / Zero 2 W (BCM SoC GPIO)
        "/dev/gpiochip10",  // Pi 5 40-pin header (RP1, common)
        "/dev/gpiochip11",  // Pi 5 additional lines on some images
        "/dev/gpiochip4",   // Alternate numbering on some kernels/images
        "/dev/gpiochip1",
    };

    if (!user_chip_trimmed.empty()) {
        std::string path = normalize_gpio_chip_path(user_chip_trimmed);
        try {
            return ::gpiod::chip(path);
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "failed to open GPIO chip '" + path + "': " + std::string(e.what()) +
                ". Run `ls /dev/gpiochip*` on the robot to pick the correct device. "
                "If the module runs in Docker, pass that device (e.g. --device /dev/gpiochip0).");
        }
    }

    std::string last_error;
    for (const char* path : k_pi_auto_candidates) {
        try {
            return ::gpiod::chip(path);
        } catch (const std::exception& e) {
            last_error = e.what();
        }
    }

    throw std::runtime_error(
        "could not open any default GPIO chip (tried /dev/gpiochip0, /dev/gpiochip10, "
        "/dev/gpiochip11, /dev/gpiochip4, /dev/gpiochip1). Last error: " +
        last_error +
        ". Set \"gpio_chip\" to the basename or full path from `ls /dev/gpiochip*`. "
        "In Docker, mount the matching device(s) into the container.");
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

    (void)pin_attr_to_offset(attrs, "step_pin");
    (void)pin_attr_to_offset(attrs, "dir_pin");
    auto enn_it = attrs.find("enn_pin");
    if (enn_it != attrs.end()) {
        (void)pin_attr_to_offset(attrs, "enn_pin");
    }

    auto spr = require_number_attr(attrs, "steps_per_revolution");
    if (spr <= 0) {
        throw std::invalid_argument("steps_per_revolution must be positive");
    }

    return {};
}

Tmc2209::Tmc2209(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg)
    : Motor(cfg.name()), has_enn_(false), steps_per_revolution_(0), max_rpm_(0) {
    auto attrs = cfg.attributes();

    ::gpiod::chip chip =
        open_gpio_chip(trim_copy(optional_string_attr(attrs, "gpio_chip", "")));

    step_offset_ = pin_attr_to_offset(attrs, "step_pin");
    dir_offset_ = pin_attr_to_offset(attrs, "dir_pin");

    auto enn_it = attrs.find("enn_pin");
    if (enn_it != attrs.end()) {
        enn_offset_ = pin_attr_to_offset(attrs, "enn_pin");
        has_enn_ = true;
    }

    steps_per_revolution_ = static_cast<int>(require_number_attr(attrs, "steps_per_revolution"));
    max_rpm_ = optional_number_attr(attrs, "max_rpm", 0);

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
