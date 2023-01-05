#include "select_avoid_object.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

SelectAvoidObject::SelectAvoidObject(const rclcpp::NodeOptions & node_options)
: Node("select_avoid_object", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  avoid_lanelet_distance_ = declare_parameter("avoid_lanelet_distance", 25);
  sub_lanelet_bin_map_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&SelectAvoidObject::onLaneletMapBin, this, _1));

  objects_sub_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/input/objects", rclcpp::QoS{10}, std::bind(&SelectAvoidObject::objectsCallback, this, _1));
  dynamic_object_pub_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/output/objects", rclcpp::QoS{1});
  rect_timer_ =
    this->create_wall_timer(second_type(5.0), std::bind(&SelectAvoidObject::onTimer, this));
}

void SelectAvoidObject::objectsCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::SharedPtr msg)
{
  in_objects_ptr_ = std::make_unique<autoware_perception_msgs::msg::DynamicObjectArray>(*msg);
  autoware_perception_msgs::msg::DynamicObjectArray filtered_objects;
  filtered_objects.header = in_objects_ptr_->header;
  for (auto object : in_objects_ptr_->objects) {
    Point object_center = {
      object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y};
    for (auto react : avoid_rects) {
      if (IsInConvexPolygon(object_center, react)) {
        filtered_objects.objects.push_back(object);
        break;
      }
    }
  }
  dynamic_object_pub_->publish(filtered_objects);
}

void SelectAvoidObject::onTimer()
{
  // get base_link position
  geometry_msgs::msg::TransformStamped map_to_baselink;
  try {
    map_to_baselink = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    return;
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = map_to_baselink.transform.translation.x;
  pose.position.y = map_to_baselink.transform.translation.y;
  Point base_link = {pose.position.x, pose.position.y};
  this->avoid_rects.clear();
  // convert lanelets to avoid area
  // check avoid polygon in meaningful distance and push
  for (const auto & ll : road_lanelets) {
    if (ll.hasAttribute("avoid")) {
      auto avoid = ll.attribute("avoid").value();
      if (avoid == "true") {
        lanelet::ConstLineString2d left = ll.leftBound2d();
        lanelet::ConstLineString2d right = ll.rightBound2d();
        if (left.size() != right.size()) {
          RCLCPP_ERROR(
            rclcpp::get_logger("SelectAvoidObject"), "lanelet left and right size not equal");
          continue;
        }
        for (int i = 0; i < (int)left.size() - 1; i++) {
          Point x1 = {left[i].x(), left[i].y()};
          Point x2 = {left[i + 1].x(), left[i + 1].y()};
          Point x3 = {right[i + 1].x(), right[i + 1].y()};
          Point x4 = {right[i].x(), right[i].y()};
          Rect rect = {x1, x2, x3, x4};
          if (
            withinDistance(base_link, x1, this->avoid_lanelet_distance_) ||
            withinDistance(base_link, x2, this->avoid_lanelet_distance_) ||
            withinDistance(base_link, x3, this->avoid_lanelet_distance_) ||
            withinDistance(base_link, x4, this->avoid_lanelet_distance_)) {
              this->avoid_rects.push_back(rect);
            } else if (IsInConvexPolygon(base_link, rect)) {
              this->avoid_rects.push_back(rect);
            }
        }
      }
    }
  }
}

void SelectAvoidObject::onLaneletMapBin(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
  this->road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SelectAvoidObject)
