
//Adding temperature later
//Add diagnostic_updater (health + rate)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <memory>
#include <string>

#include "imu/imu.hpp"
#include "imu/ism330dhcx.hpp"
#include "imu/icm20948.hpp"

using namespace std::chrono_literals;

class ImuPublisherNode : public rclcpp::Node
{
public:
    ImuPublisherNode() : Node("imu_publisher")
    {
        declare_parameters();
        setup_imus();

        timer_ = create_wall_timer(
            10ms, std::bind(&ImuPublisherNode::timer_callback, this));
    }

private:
    /* ---------------- Parameters ---------------- */

    struct ImuConfig
	{
        bool enabled{false};
        std::string i2c_bus;
        int address{0};
        std::string frame_id;
        std::string topic;
        std::string mag_topic;      
    };

    ImuConfig ism_cfg_;
    ImuConfig icm_cfg_;

    /* ---------------- Driver ---------------- */

    std::unique_ptr<ImuDriver> ism_driver_;
    std::unique_ptr<ImuDriver> icm_driver_;

    /* ---------------- Publisher ---------------- */

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ism_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr icm_pub_;

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr icm_mag_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* ---------------- Setup ---------------- */
	/* default, override in yaml file */

    void declare_parameters()
    {
        // ISM330
        declare_parameter("ism330.enabled", true);
        declare_parameter("ism330.i2c_bus", "/dev/i2c-1");
        declare_parameter("ism330.address", 0x6B);
		declare_parameter("ism330.topic", "/imu1");
		declare_parameter("ism330.frame_id", "imu1_link");

        // ICM20948
        declare_parameter("icm20948.enabled", false);
        declare_parameter("icm20948.i2c_bus", "/dev/i2c-1");
        declare_parameter("icm20948.address", 0x69);
		declare_parameter("icm20948.topic", "/imu2");
		declare_parameter("icm20948.frame_id", "imu2_link");
		declare_parameter("icm20948.mag_topic", "/imu2/mag");
    }

    void setup_imus()
    {
        /* ---------- ISM330 ---------- */
        ism_cfg_.enabled = get_parameter("ism330.enabled").as_bool();
        ism_cfg_.i2c_bus = get_parameter("ism330.i2c_bus").as_string();
        ism_cfg_.address = get_parameter("ism330.address").as_int();
        //ism_cfg_.frame_id = "ism330_imu_link";
        //ism_cfg_.topic = "/imu/ism330/data_raw";
		ism_cfg_.frame_id =
		  get_parameter("ism330.frame_id").as_string();
		ism_cfg_.topic =
		  get_parameter("ism330.topic").as_string();
		icm_cfg_.mag_topic =
		  get_parameter("icm20948.mag_topic").as_string();

        if (ism_cfg_.enabled) {
            ism_driver_ = std::make_unique<ISM330DHCXDriver>(
                ism_cfg_.i2c_bus, static_cast<uint8_t>(ism_cfg_.address));

            if (!ism_driver_->initialize()) {
                RCLCPP_ERROR(get_logger(), "Failed to initialize ISM330DHCX");
                ism_driver_.reset();
            } else {
                ism_pub_ = create_publisher<sensor_msgs::msg::Imu>(
                    ism_cfg_.topic, 10);
                RCLCPP_INFO(get_logger(), "ISM330DHCX enabled");
            }
        }

        /* ---------- ICM20948 ---------- */
        icm_cfg_.enabled = get_parameter("icm20948.enabled").as_bool();
        icm_cfg_.i2c_bus = get_parameter("icm20948.i2c_bus").as_string();
        icm_cfg_.address = get_parameter("icm20948.address").as_int();
        //icm_cfg_.frame_id = "icm20948_imu_link";
        //icm_cfg_.topic = "/imu/icm20948/data_raw";
		icm_cfg_.frame_id =
		  get_parameter("icm20948.frame_id").as_string();
		icm_cfg_.topic =
		  get_parameter("icm20948.topic").as_string();


        if (icm_cfg_.enabled)
		{
            icm_driver_ = std::make_unique<ICM20948Driver>(
                icm_cfg_.i2c_bus, static_cast<uint8_t>(icm_cfg_.address));

            if (!icm_driver_->initialize())
			{
                RCLCPP_ERROR(get_logger(), "Failed to initialize ICM20948");
                icm_driver_.reset();
            } else
			{
                icm_pub_ = create_publisher<sensor_msgs::msg::Imu>(
                    icm_cfg_.topic, 10);

                if (icm_driver_->has_magnetometer())
				{
					icm_mag_pub_ =
						create_publisher<sensor_msgs::msg::MagneticField>(
						icm_cfg_.mag_topic, 10);
						}
                RCLCPP_INFO(get_logger(), "ICM20948 enabled");
            }
        }
    }

    /* ---------------- Timer ---------------- */

    void timer_callback()
    {
        if (ism_driver_ && ism_pub_)
		{
            publish_imu(*ism_driver_, ism_pub_, ism_cfg_.frame_id);
        }

        if (icm_driver_ && icm_pub_)
		{
            publish_imu(*icm_driver_, icm_pub_, icm_cfg_.frame_id);

            if (icm_driver_->has_magnetometer() && icm_mag_pub_)
			{
                publish_mag(*icm_driver_, icm_cfg_.frame_id);
            }
        }
    }

    /* ---------------- Publish Helpers ---------------- */

    void publish_imu(ImuDriver &driver,
                     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
                     const std::string &frame_id)
    {
        ImuSample sample;
        if (!driver.read(sample))
		{
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;

        msg.linear_acceleration.x = sample.ax;
        msg.linear_acceleration.y = sample.ay;
        msg.linear_acceleration.z = sample.az;

        msg.angular_velocity.x = sample.gx;
        msg.angular_velocity.y = sample.gy;
        msg.angular_velocity.z = sample.gz;

        pub->publish(msg);
    }

    void publish_mag(ImuDriver &driver, const std::string &frame_id)
    {
        MagSample mag;
        if (!driver.read_magnetometer(mag))
		{
            return;
        }

        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id;

        msg.magnetic_field.x = mag.mx;
        msg.magnetic_field.y = mag.my;
        msg.magnetic_field.z = mag.mz;

        icm_mag_pub_->publish(msg);
    }
};

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
