#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class GpsNode : public rclcpp::Node
{
public:
    GpsNode() : Node("gps_node")
    {
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        raw_pub_ = this->create_publisher<std_msgs::msg::String>("gps/raw", 10);

        // Open serial port (simple version)
        serial_.open("/dev/ttyAMA5");
        if (!serial_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyAMA5");
            return;
        }

        // Timer to read periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&GpsNode::readSerial, this));
    }

private:
    void readSerial()
    {
        std::string line;
        if (!std::getline(serial_, line))
            return;

        // Publish raw NMEA
        std_msgs::msg::String raw_msg;
        raw_msg.data = line;
        raw_pub_->publish(raw_msg);

        // If it's a GGA sentence, parse it
        if (line.rfind("$GNGGA", 0) == 0) {
            auto fix_msg = parseGGA(line);
            if (fix_msg) {
                fix_pub_->publish(*fix_msg);
            }
        }
    }

    std::optional<sensor_msgs::msg::NavSatFix> parseGGA(const std::string &nmea)
    {
        std::stringstream ss(nmea);
        std::string token;
        std::vector<std::string> fields;
        while (std::getline(ss, token, ',')) {
            fields.push_back(token);
        }

        if (fields.size() < 10) return std::nullopt;

        try {
            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "gps";

            // Latitude
            double lat_deg = std::stod(fields[2].substr(0,2));
            double lat_min = std::stod(fields[2].substr(2));
            double lat = lat_deg + lat_min/60.0;
            if (fields[3] == "S") lat = -lat;

            // Longitude
            double lon_deg = std::stod(fields[4].substr(0,3));
            double lon_min = std::stod(fields[4].substr(3));
            double lon = lon_deg + lon_min/60.0;
            if (fields[5] == "W") lon = -lon;

            msg.latitude = lat;
            msg.longitude = lon;
            msg.altitude = std::stod(fields[9]);

            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            return msg;
        } catch (...) {
            return std::nullopt;
        }
    }

    std::ifstream serial_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNode>());
    rclcpp::shutdown();
    return 0;
}
