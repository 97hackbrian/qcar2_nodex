/**
 * @file buzzer_alert.cpp
 * @brief ROS 2 node that monitors QCar2 battery voltage and triggers
 *        audio alerts through the Tegra DSPK (Digital Speaker).
 *        Optionally monitors camera topics for inactivity.
 *
 * The QCar2 buzzer is the Jetson Orin's DSPK, reached via ALSA:
 *   1. Configure mixer route: DSPK1 Mux → ADMAIF1
 *   2. Play WAV through hw:1,0  (APE card, ADMAIF1 device)
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

// ---------------------------------------------------------------------------
// WAV generator
// ---------------------------------------------------------------------------

struct WavHeader {
    char     riff[4]        = {'R','I','F','F'};
    uint32_t overall_size   = 0;
    char     wave[4]        = {'W','A','V','E'};
    char     fmt_marker[4]  = {'f','m','t',' '};
    uint32_t fmt_length     = 16;
    uint16_t format_type    = 1;          // PCM
    uint16_t channels       = 1;          // mono
    uint32_t sample_rate    = 48000;
    uint32_t byterate       = 96000;      // sr * ch * bps/8
    uint16_t block_align    = 2;          // ch * bps/8
    uint16_t bits_per_sample= 16;
    char     data_marker[4] = {'d','a','t','a'};
    uint32_t data_size      = 0;
};

static bool generate_wav(const std::string &path,
                         int beeps, double freq_hz,
                         int on_ms, int off_ms,
                         double volume = 0.8)
{
    const uint32_t sr = 48000;
    const int s_on  = static_cast<int>(sr * on_ms  / 1000);
    const int s_off = static_cast<int>(sr * off_ms / 1000);
    const int total = beeps * (s_on + s_off);
    std::vector<int16_t> pcm(total, 0);
    for (int b = 0; b < beeps; ++b) {
        int off = b * (s_on + s_off);
        for (int s = 0; s < s_on; ++s) {
            double t = static_cast<double>(s) / sr;
            pcm[off + s] = static_cast<int16_t>(
                volume * std::sin(2.0 * M_PI * freq_hz * t) * 32767.0);
        }
    }
    WavHeader h;
    h.sample_rate = sr;
    h.byterate    = sr * 1 * 2;
    h.data_size   = total * sizeof(int16_t);
    h.overall_size = h.data_size + sizeof(WavHeader) - 8;
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return false;
    f.write(reinterpret_cast<const char*>(&h), sizeof(h));
    f.write(reinterpret_cast<const char*>(pcm.data()), h.data_size);
    return true;
}

// ---------------------------------------------------------------------------
// BuzzerAlert node
// ---------------------------------------------------------------------------

class BuzzerAlert : public rclcpp::Node
{
public:
    BuzzerAlert()
    : Node("buzzer_alert")
    {
        // ---- Declare parameters ----
        this->declare_parameter("battery_threshold_percent", 40.0);
        this->declare_parameter("battery_critical_percent",  20.0);
        this->declare_parameter("battery_voltage_max",       12.6);
        this->declare_parameter("battery_voltage_min",       10.0);
        this->declare_parameter("check_interval_sec",         5.0);
        this->declare_parameter("enable_camera_watchdog",    false);
        this->declare_parameter("camera_topic",
                                std::string("/camera/csi_image_0"));
        this->declare_parameter("camera_timeout_sec",        10.0);
        this->declare_parameter("alsa_device",
                                std::string("hw:1,0"));

        // ---- Read parameters ----
        threshold_pct_  = this->get_parameter("battery_threshold_percent")
                              .as_double();
        critical_pct_   = this->get_parameter("battery_critical_percent")
                              .as_double();
        voltage_max_    = this->get_parameter("battery_voltage_max")
                              .as_double();
        voltage_min_    = this->get_parameter("battery_voltage_min")
                              .as_double();
        check_interval_ = this->get_parameter("check_interval_sec")
                              .as_double();
        camera_watchdog_= this->get_parameter("enable_camera_watchdog")
                              .as_bool();
        camera_topic_   = this->get_parameter("camera_topic")
                              .as_string();
        camera_timeout_ = this->get_parameter("camera_timeout_sec")
                              .as_double();
        alsa_device_    = this->get_parameter("alsa_device")
                              .as_string();

        // ---- Configure DSPK ALSA routing ----
        configure_dspk();

        // ---- Generate WAV tone files ----
        bool ok = true;
        ok &= generate_wav("/tmp/buzzer_low.wav",  3, 1000, 200, 200, 0.9);
        ok &= generate_wav("/tmp/buzzer_crit.wav", 6, 1500, 100, 100, 1.0);
        ok &= generate_wav("/tmp/buzzer_cam.wav",  2,  600, 500, 300, 0.7);
        if (!ok) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to generate one or more WAV files");
        }

        // ---- Subscribers ----
        battery_sub_ = this->create_subscription<
            sensor_msgs::msg::BatteryState>(
                "qcar2_battery", 10,
                std::bind(&BuzzerAlert::battery_callback, this, _1));

        if (camera_watchdog_) {
            camera_sub_ = this->create_subscription<
                sensor_msgs::msg::Image>(
                    camera_topic_, 1,
                    std::bind(&BuzzerAlert::camera_callback, this, _1));
            last_camera_time_ = this->now();
            RCLCPP_INFO(this->get_logger(),
                        "Camera watchdog enabled on: %s",
                        camera_topic_.c_str());
        }

        // ---- Periodic check timer ----
        auto interval = std::chrono::duration<double>(check_interval_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(interval),
            std::bind(&BuzzerAlert::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "buzzer_alert started | threshold=%.0f%% (%.2fV) | "
            "critical=%.0f%% (%.2fV) | check every %.1fs | device=%s",
            threshold_pct_,
            pct_to_voltage(threshold_pct_),
            critical_pct_,
            pct_to_voltage(critical_pct_),
            check_interval_,
            alsa_device_.c_str());
    }

    ~BuzzerAlert()
    {
        stop_buzzer_loop();
    }

private:

    // ---- Configure Tegra DSPK ALSA mixer routing ----
    void configure_dspk()
    {
        RCLCPP_INFO(this->get_logger(), "Configuring DSPK ALSA routing...");

        const char* cmds[] = {
            "amixer cset name='DSPK1 Mux' 'ADMAIF1' 2>/dev/null",
            "amixer cset name='DSPK1 Audio Bit Format' '16' 2>/dev/null",
            "amixer cset name='DSPK1 Audio Channels' 2 2>/dev/null",
            "amixer cset name='DSPK1 Mono To Stereo' 'Copy' 2>/dev/null",
            nullptr
        };

        for (int i = 0; cmds[i] != nullptr; ++i) {
            int ret = std::system(cmds[i]);
            if (ret != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "DSPK config command returned %d: %s", ret, cmds[i]);
            }
        }

        RCLCPP_INFO(this->get_logger(),
                     "DSPK routing configured (ADMAIF1 → DSPK1)");
    }

    // ---- Voltage <-> percentage ----

    double voltage_to_pct(double v) const {
        if (voltage_max_ <= voltage_min_) return 0.0;
        double pct = (v - voltage_min_)
                   / (voltage_max_ - voltage_min_) * 100.0;
        return std::clamp(pct, 0.0, 100.0);
    }

    double pct_to_voltage(double pct) const {
        return voltage_min_
             + (pct / 100.0) * (voltage_max_ - voltage_min_);
    }

    // ---- ROS callbacks ----

    void battery_callback(
        const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        latest_voltage_ = msg->voltage;
        battery_received_ = true;
    }

    void camera_callback(
        const sensor_msgs::msg::Image::SharedPtr /*msg*/)
    {
        last_camera_time_ = this->now();
        camera_received_ = true;
    }

    void timer_callback()
    {
        if (!battery_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 30000,
                                 "No battery data received yet");
            return;
        }

        double pct = voltage_to_pct(latest_voltage_);

        // ---- Determine alert level ----
        AlertLevel desired = AlertLevel::NONE;

        if (pct < critical_pct_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                                  10000,
                "CRITICAL BATTERY: %.1f%% (%.2fV) — Charge immediately!",
                pct, latest_voltage_);
            desired = AlertLevel::CRITICAL;
        }
        else if (pct < threshold_pct_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 30000,
                "Low battery: %.1f%% (%.2fV)", pct, latest_voltage_);
            desired = AlertLevel::LOW;
        }

        // ---- Camera watchdog ----
        if (camera_watchdog_ && desired == AlertLevel::NONE) {
            bool cam_alert = false;
            if (!camera_received_) {
                cam_alert = true;
            } else {
                double elapsed =
                    (this->now() - last_camera_time_).seconds();
                if (elapsed > camera_timeout_) cam_alert = true;
            }
            if (cam_alert) {
                RCLCPP_WARN_THROTTLE(this->get_logger(),
                    *this->get_clock(), 30000,
                    "Camera watchdog: no images on %s",
                    camera_topic_.c_str());
                desired = AlertLevel::CAMERA;
            }
        }

        // ---- Start or stop continuous buzzer ----
        if (desired != current_alert_) {
            stop_buzzer_loop();  // stop previous sound
            if (desired != AlertLevel::NONE) {
                const char* wav = "/tmp/buzzer_low.wav";
                if (desired == AlertLevel::CRITICAL)
                    wav = "/tmp/buzzer_crit.wav";
                else if (desired == AlertLevel::CAMERA)
                    wav = "/tmp/buzzer_cam.wav";
                start_buzzer_loop(wav);
            }
            current_alert_ = desired;
        }

        // If buzzer should be playing, check it's still alive
        if (current_alert_ != AlertLevel::NONE && !is_buzzer_playing()) {
            const char* wav = "/tmp/buzzer_low.wav";
            if (current_alert_ == AlertLevel::CRITICAL)
                wav = "/tmp/buzzer_crit.wav";
            else if (current_alert_ == AlertLevel::CAMERA)
                wav = "/tmp/buzzer_cam.wav";
            start_buzzer_loop(wav);
        }
    }

    // ---- Continuous buzzer loop management ----

    /**
     * Start a background bash loop that continuously plays the WAV file
     * with a pause between each repetition.
     */
    void start_buzzer_loop(const char* wav_path)
    {
        stop_buzzer_loop();  // ensure no stale process

        // Spawn: while true; do aplay ...; sleep 1; done
        // The subshell writes its PID to a file so we can kill it
        std::string cmd =
            "bash -c 'echo $$ > /tmp/buzzer_loop.pid; "
            "while true; do "
            "aplay -q -D " + alsa_device_ + " " + wav_path + " 2>/dev/null; "
            "sleep 1; "
            "done' &";

        std::system(cmd.c_str());

        RCLCPP_INFO(this->get_logger(),
            "Buzzer loop started: %s", wav_path);
    }

    /**
     * Kill the background buzzer loop if running.
     */
    void stop_buzzer_loop()
    {
        // Read PID from file and kill the process group
        std::ifstream pf("/tmp/buzzer_loop.pid");
        if (pf.is_open()) {
            int pid = 0;
            pf >> pid;
            pf.close();
            if (pid > 0) {
                // Kill the entire process group (bash + aplay children)
                std::string cmd = "kill -- -" + std::to_string(pid)
                                + " 2>/dev/null; "
                                  "kill " + std::to_string(pid)
                                + " 2>/dev/null; "
                                  "pkill -P " + std::to_string(pid)
                                + " 2>/dev/null";
                std::system(cmd.c_str());
            }
            std::remove("/tmp/buzzer_loop.pid");
        }
    }

    /**
     * Check if the buzzer loop process is still running.
     */
    bool is_buzzer_playing()
    {
        std::ifstream pf("/tmp/buzzer_loop.pid");
        if (!pf.is_open()) return false;
        int pid = 0;
        pf >> pid;
        pf.close();
        if (pid <= 0) return false;

        // Check if process exists
        std::string path = "/proc/" + std::to_string(pid);
        struct stat st;
        return (stat(path.c_str(), &st) == 0);
    }

    // ---- Members ----

    enum class AlertLevel { NONE, LOW, CRITICAL, CAMERA };

    // Parameters
    double threshold_pct_;
    double critical_pct_;
    double voltage_max_;
    double voltage_min_;
    double check_interval_;
    bool   camera_watchdog_;
    std::string camera_topic_;
    double camera_timeout_;
    std::string alsa_device_;

    // State
    double latest_voltage_   = 0.0;
    bool   battery_received_ = false;
    bool   camera_received_  = false;
    AlertLevel current_alert_ = AlertLevel::NONE;
    rclcpp::Time last_camera_time_{0, 0, RCL_ROS_TIME};

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
        battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        camera_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuzzerAlert>();
    RCLCPP_INFO(node->get_logger(), "buzzer_alert node spinning...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "buzzer_alert node shutting down.");
    rclcpp::shutdown();
    return 0;
}
