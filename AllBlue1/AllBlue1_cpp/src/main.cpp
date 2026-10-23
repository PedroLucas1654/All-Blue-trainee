// Nó ROS 2: lê o GPS GY-NEO6MV2 pela serial e publica
//   gps/fix  (sensor_msgs/NavSatFix)
//   gps/vel  (geometry_msgs/TwistStamped, ENU em m/s)
#include <AllBlue1_cpp/core.hpp>
#include <AllBlue1_cpp/AllBlue1.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

using geometry_msgs::msg::TwistStamped;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

class GpsNode : public rclcpp::Node {
public:
    GpsNode() : Node("gps_node") {
        const auto device = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        const int baud = declare_parameter<int>("baud", 9600);
        frame_id_ = declare_parameter<std::string>("frame_id", "gps_link");
        hdop_to_sigma_m_ = declare_parameter<double>("hdop_to_sigma_m", 2.5);

        fix_pub_ = create_publisher<NavSatFix>("gps/fix", 10);
        vel_pub_ = create_publisher<TwistStamped>("gps/vel", 10);

        if (!port_.open(device, baud)) {
            throw std::runtime_error("não foi possível abrir a porta " + device);
        }
        RCLCPP_INFO(get_logger(), "Lendo GPS em %s @ %d baud", device.c_str(), baud);

        running_ = true;
        thread_ = std::thread(&GpsNode::read_loop, this);
    }

    ~GpsNode() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();  // read_line tem timeout de ~1 s
    }

private:
    // Thread própria: read_line() bloqueia até 1 s e não pode travar o executor
    void read_loop() {
        std::string line;
        while (running_ && rclcpp::ok()) {
            if (!port_.read_line(line)) continue;
            // RMC chega 1x por ciclo; GGA já foi processada antes e fica acumulada
            if (parser_.feed_line(line) == allblue1::Sentence::RMC) publish(parser_.data());
        }
    }

    void publish(const allblue1::GpsData& d) {
        const auto stamp = now();  // hora do PC; o tempo UTC do GPS está em d.utc_time

        NavSatFix fix;
        fix.header.stamp = stamp;
        fix.header.frame_id = frame_id_;
        fix.status.service = NavSatStatus::SERVICE_GPS;

        if (d.has_fix) {
            fix.status.status = NavSatStatus::STATUS_FIX;
            fix.latitude = d.latitude;
            fix.longitude = d.longitude;
            fix.altitude = d.altitude_m;
            // Covariância = ESTIMATIVA, não garantia: sigma ≈ HDOP * hdop_to_sigma_m.
            // O padrão (2,5 m) vem do valor de CEP citado no datasheet do NEO-6 em
            // condições ideais; o erro real depende do céu visível e do multipath.
            // Calibre com dados reais ou use hdop_to_sigma_m:=0 para declarar "desconhecida".
            if (hdop_to_sigma_m_ > 0.0) {
                const double sh = std::max(d.hdop, 1.0) * hdop_to_sigma_m_;
                const double sv = sh * 2.0;
                fix.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
                fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
            } else {
                fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            }
        } else {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            fix.status.status = NavSatStatus::STATUS_NO_FIX;
            fix.latitude = fix.longitude = fix.altitude = nan;
            fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }
        fix_pub_->publish(fix);

        if (d.has_fix) {
            // curso é medido a partir do Norte, no sentido horário
            const double v = d.speed_kmh / 3.6;
            const double c = d.course_deg * M_PI / 180.0;
            TwistStamped vel;
            vel.header = fix.header;
            vel.twist.linear.x = v * std::sin(c);  // Leste
            vel.twist.linear.y = v * std::cos(c);  // Norte
            vel_pub_->publish(vel);
        }
    }

    allblue1::SerialPort port_;
    allblue1::NmeaParser parser_;
    std::string frame_id_;
    double hdop_to_sigma_m_ = 2.5;
    rclcpp::Publisher<NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr vel_pub_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        rclcpp::spin(std::make_shared<GpsNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("gps_node"), "%s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
