#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GnssSubscriber : public rclcpp::Node
{
public:
  GnssSubscriber()
  : Node("gnss_subscriber_node")
  {
    // 最新のデータを最優先で受け取る設定 (Best Effort)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // u-bloxドライバが配信するトピックをサブスクライブ
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix", qos,
      std::bind(&GnssSubscriber::topic_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "GNSS Subscriber Node has been started.");
  }

private:
  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
  {
    int status = msg->status.status;
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    double altitude = msg->altitude;

    // ターミナルに受信データを表示
    RCLCPP_INFO(this->get_logger(), "Status: %d | Lat: %.7f, Lon: %.7f, Alt: %.2fm", 
                status, latitude, longitude, altitude);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssSubscriber>());
  rclcpp::shutdown();
  return 0;
}
