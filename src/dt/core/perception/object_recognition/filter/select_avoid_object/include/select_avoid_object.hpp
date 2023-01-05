#ifndef SELECT_AVOID_OBJECT_HPP_
#define SELECT_AVOID_OBJECT_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Point
{
  double x;
  double y;
};

struct Rect
{
  Point x1;
  Point x2;
  Point x3;
  Point x4;
};

class SelectAvoidObject : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit SelectAvoidObject(const rclcpp::NodeOptions & options);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double avoid_lanelet_distance_;
  rclcpp::TimerBase::SharedPtr   rect_timer_;
  lanelet::ConstLanelets road_lanelets;
  std::vector<Rect> avoid_rects;

  std::unique_ptr<autoware_perception_msgs::msg::DynamicObjectArray> in_objects_ptr_;

  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr sub_lanelet_bin_map_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    dynamic_object_pub_;

  /// \brief callback for loading lanelet2 map
  void onLaneletMapBin(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);

  void objectsCallback(const autoware_perception_msgs::msg::DynamicObjectArray::SharedPtr msg);

  void onTimer();
  static bool IsInConvexPolygon(Point testPoint, Rect polygon)
  {
    // n>2 Keep track of cross product sign changes
    short pos = 0;
    short neg = 0;

    // If point is in the polygon
    if (testPoint.x == polygon.x1.x && testPoint.y == polygon.x1.y) return true;
    if (testPoint.x == polygon.x2.x && testPoint.y == polygon.x2.y) return true;
    if (testPoint.x == polygon.x3.x && testPoint.y == polygon.x3.y) return true;
    if (testPoint.x == polygon.x4.x && testPoint.y == polygon.x4.y) return true;

    // TODO(jiheng): should loop smarter
    auto d12 =
      Product(polygon.x1.x, polygon.x1.y, polygon.x2.x, polygon.x2.y, testPoint.x, testPoint.y);
    if (d12 > 0) pos++;
    if (d12 < 0) neg++;
    if (pos > 0 && neg > 0) return false;

    auto d23 =
      Product(polygon.x2.x, polygon.x2.y, polygon.x3.x, polygon.x3.y, testPoint.x, testPoint.y);
    if (d23 > 0) pos++;
    if (d23 < 0) neg++;
    if (pos > 0 && neg > 0) return false;

    auto d34 =
      Product(polygon.x3.x, polygon.x3.y, polygon.x4.x, polygon.x4.y, testPoint.x, testPoint.y);
    if (d34 > 0) pos++;
    if (d34 < 0) neg++;
    if (pos > 0 && neg > 0) return false;

    auto d41 =
      Product(polygon.x4.x, polygon.x4.y, polygon.x1.x, polygon.x1.y, testPoint.x, testPoint.y);
    if (d41 > 0) pos++;
    if (d41 < 0) neg++;
    if (pos > 0 && neg > 0) return false;
    return true;
  }

  static double Product(double x1, double y1, double x2, double y2, double cx, double cy)
  {
    // Compute the cross product
    double d = (cx - x1) * (y2 - y1) - (cy - y1) * (x2 - x1);
    return d;
  }

  static bool withinDistance(Point point1, Point point2, double distance)
  {
    double delta =
      (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y);
    return delta <= distance * distance;
  }
};

#endif
