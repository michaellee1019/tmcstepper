#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/board.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>

namespace tmcstepper {

class Tmc2209 : public viam::sdk::Motor {
   public:
    Tmc2209(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg);
    ~Tmc2209();

    static std::vector<std::string> validate(const viam::sdk::ResourceConfig& cfg);

    void stop(const viam::sdk::ProtoStruct& extra) override;

    void set_power(double power_pct, const viam::sdk::ProtoStruct& extra) override;

    void go_for(double rpm, double revolutions, const viam::sdk::ProtoStruct& extra) override;

    void go_to(double rpm, double position_revolutions,
               const viam::sdk::ProtoStruct& extra) override;

    void set_rpm(double rpm, const viam::sdk::ProtoStruct& extra) override;

    void reset_zero_position(double offset, const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::Motor::position get_position(const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::Motor::properties get_properties(const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::Motor::power_status get_power_status(const viam::sdk::ProtoStruct& extra) override;

    bool is_moving() override;

    viam::sdk::ProtoStruct get_status() override;

    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;

    std::vector<viam::sdk::GeometryConfig> get_geometries(
        const viam::sdk::ProtoStruct& extra) override;

   private:
    uint64_t rpm_to_freq_hz(double rpm) const;
    void start_pwm(bool forward, uint64_t freq_hz);
    void stop_hardware();
    void cancel_tracking();
    void track_position(int64_t target, bool forward, double freq_hz);

    std::shared_ptr<viam::sdk::Board> board_;
    std::string step_pin_;
    std::string dir_pin_;
    std::string enn_pin_;
    int steps_per_revolution_;
    double max_rpm_;

    std::atomic<int64_t> step_position_{0};
    std::atomic<int64_t> target_step_position_{0};

    std::mutex lock_;
    std::atomic<bool> cancel_flag_{false};
    std::thread tracking_thread_;
};

}  // namespace tmcstepper
