#include <memory>
#include <cmath>
#include <functional> // Jazzy（GCC 13以降）のバインド・プレースホルダー用
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GnssSubscriber : public rclcpp::Node
{
public:
  GnssSubscriber()
  : Node("gnss_subscriber_node"), is_origin_set_(false)
  {
    // 最新データを最優先にする低遅延QoS設定
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // トピックのサブスクライブ
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix", qos,
      std::bind(&GnssSubscriber::topic_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "GNSS ENU Converter Node has been started.");
  }

private:
  // 地球の形状を表す WGS84 楕円体の定数
  const double WGS84_A = 6378137.0;          // 赤道半径 (m)
  const double WGS84_E2 = 0.00669437999014;  // 第一離心率の二乗

  // 基準点（ロボットカーのスタート位置）の管理用変数
  bool is_origin_set_;
  double lat0_, lon0_, alt0_;
  double x0_ecef_, y0_ecef_, z0_ecef_;

  // 【理論式1】 LLA（緯度・経度・高度）を ECEF（地球中心直交座標系）に変換
  void lla_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z) const
  {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    
    // 卯酉線（ぼうゆうせん）曲率半径の計算
    double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));
    
    x = (n + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    y = (n + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    z = (n * (1.0 - WGS84_E2) + alt) * std::sin(lat_rad);
  }

  // 【理論式2】 ECEF座標から基準点ベースの ENU（東・北・上）メートル座標に変換
  void ecef_to_enu(double x, double y, double z, double& east, double& north, double& up) const
  {
    double lat0_rad = lat0_ * M_PI / 180.0;
    double lon0_rad = lon0_ * M_PI / 180.0;

    // 基準点（スタート位置）からの位置の差分（メートル）
    double dx = x - x0_ecef_;
    double dy = y - y0_ecef_;
    double dz = z - z0_ecef_;

    // 回転行列による座標変換
    east  = -std::sin(lon0_rad) * dx + std::cos(lon0_rad) * dy;
    north = -std::sin(lat0_rad) * std::cos(lon0_rad) * dx - std::sin(lat0_rad) * std::sin(lon0_rad) * dy + std::cos(lat0_rad) * dz;
    up    =  std::cos(lat0_rad) * std::cos(lon0_rad) * dx + std::cos(lat0_rad) * std::sin(lon0_rad) * dy + std::sin(lat0_rad) * dz;
  }

  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // アンテナが衛星を捉えられていない（NO_FIX）場合は処理をスキップ
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_WARN(this->get_logger(), "Waiting for GNSS Fix (Satellite signal)...");
      return;
    }

    // 現在の位置の緯度経度を、一度ECEF(X, Y, Z)に変換
    double x_ecef, y_ecef, z_ecef;
    lla_to_ecef(msg->latitude, msg->longitude, msg->altitude, x_ecef, y_ecef, z_ecef);

    // 起動直後、一番最初に取得した高精度な1点を「原点 (0, 0, 0)」に設定
    if (!is_origin_set_) {
      lat0_ = msg->latitude;
      lon0_ = msg->longitude;
      alt0_ = msg->altitude;
      x0_ecef_ = x_ecef;
      y0_ecef_ = y_ecef;
      z0_ecef_ = z_ecef;
      is_origin_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Origin Base Set! -> Lat: %.7f, Lon: %.7f", lat0_, lon0_);
    }

    // 現在地を、原点からの相対メートル座標(ENU)に変換
    double east, north, up;
    ecef_to_enu(x_ecef, y_ecef, z_ecef, east, north, up);

    // 千葉工大の実験場や屋外を走行中の位置（メートル）を出力
    RCLCPP_INFO(this->get_logger(), "ENU Meter -> East(X): %.3f m, North(Y): %.3f m, Up(Z): %.3f m", 
                east, north, up);
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
