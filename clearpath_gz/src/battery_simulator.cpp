#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <cmath>

class BatterySimulator : public rclcpp::Node
{
public:
    BatterySimulator() : Node("battery_simulator")
    {
        // Clearpath Husky A200 Specs (ES20-12 Lead Acid, S2P1 configuration)
        this->declare_parameter<double>("initial_percentage", 1.0);
        this->declare_parameter<double>("nominal_voltage", 24.0);
        this->declare_parameter<double>("capacity_ah", 20.0);
        this->declare_parameter<double>("idle_current_amps", 1.25); // Baseline Husky PC + Sensors
        this->declare_parameter<double>("discharge_rate_per_meter", 0.0001); // Ah per meter
        this->declare_parameter<std::string>("odom_topic", "/odometry");
        this->declare_parameter<std::string>("battery_topic", "/battery_state");

        current_percentage_ = this->get_parameter("initial_percentage").as_double();
        nominal_voltage_ = this->get_parameter("nominal_voltage").as_double();
        capacity_ah_ = this->get_parameter("capacity_ah").as_double();
        idle_current_amps_ = this->get_parameter("idle_current_amps").as_double();
        discharge_rate_per_meter_ = this->get_parameter("discharge_rate_per_meter").as_double();
        
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string battery_topic = this->get_parameter("battery_topic").as_string();

        // Internal state: track charge in Amp-hours
        current_charge_ah_ = current_percentage_ * capacity_ah_;

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&BatterySimulator::odomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&BatterySimulator::updateAndPublish, this));

        last_update_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Husky Battery Simulator: %.1fV %zuAh system initialized.", nominal_voltage_, static_cast<size_t>(capacity_ah_));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (first_odom_) {
            last_x_ = msg->pose.pose.position.x;
            last_y_ = msg->pose.pose.position.y;
            first_odom_ = false;
            return;
        }

        double dx = msg->pose.pose.position.x - last_x_;
        double dy = msg->pose.pose.position.y - last_y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > 0.01) {
            // Discharge based on distance moved
            current_charge_ah_ -= dist * discharge_rate_per_meter_;
            last_x_ = msg->pose.pose.position.x;
            last_y_ = msg->pose.pose.position.y;
        }

        if (current_charge_ah_ < 0.0) current_charge_ah_ = 0.0;
    }

    void updateAndPublish()
    {
        auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Discharge based on idle time
            current_charge_ah_ -= (idle_current_amps_ * dt) / 3600.0;
            if (current_charge_ah_ < 0.0) current_charge_ah_ = 0.0;
            current_percentage_ = current_charge_ah_ / capacity_ah_;
        }

        auto msg = sensor_msgs::msg::BatteryState();
        msg.header.stamp = now;
        msg.header.frame_id = "base_link";
        
        // Lead-acid 24V Voltage Curve Simulation (Linear approximation 21.0V to 26.4V)
        float full_v = 26.4f;
        float empty_v = 21.0f;
        msg.voltage = empty_v + (full_v - empty_v) * static_cast<float>(current_percentage_);
        
        msg.percentage = static_cast<float>(current_percentage_);
        msg.charge = static_cast<float>(current_charge_ah_);
        msg.capacity = static_cast<float>(capacity_ah_);
        msg.design_capacity = static_cast<float>(capacity_ah_);
        msg.current = -static_cast<float>(idle_current_amps_); // Negative for discharging
        
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE; // Husky uses Lead Acid but LIFE is closest standard
        msg.present = true;

        battery_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_percentage_;
    double current_charge_ah_;
    double nominal_voltage_;
    double capacity_ah_;
    double idle_current_amps_;
    double discharge_rate_per_meter_;
    double last_x_, last_y_;
    bool first_odom_ = true;
    rclcpp::Time last_update_time_;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatterySimulator>());
    rclcpp::shutdown();
    return 0;
}
