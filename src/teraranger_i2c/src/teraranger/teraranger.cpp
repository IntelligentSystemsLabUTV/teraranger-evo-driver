/**
 * Teraranger node definition.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 13, 2023
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <string>
#include <stdexcept>

#include <teraranger/teraranger.hpp>

namespace Teraranger
{

/**
 * @brief Builds a new Teraranger node.
 *
 * @param opts Node options.
 *
 * @throws RuntimeError
 */
TerarangerNode::TerarangerNode(const rclcpp::NodeOptions &node_options)
  : NodeBase("teraranger", node_options, true)
{
  // Initialize callback groups
  init_cgroups();

  // Initialize parameters
  init_parameters();

  // Initialize topic publishers
  init_publishers();

  // Initialize I²C communication
  init_i2c();

  // Initialize timer
  init_timers();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Finalizes node operation.
 */
TerarangerNode::~TerarangerNode()
{
  RCLCPP_INFO(this->get_logger(), "Destructor called, closing communication.");
  this->end();
}

/**
 * @brief Routine to initialize topic publishers.
 */
void TerarangerNode::init_i2c()
{
  // Start serial interface and perform handshake
  if (begin(port.c_str()) < 0)
  {
    RCLCPP_FATAL(this->get_logger(), "Cannot open serial port");
    rclcpp::shutdown();
    return;
  }

  // Initialize range message
  range_msg.field_of_view = field_of_view;
  range_msg.max_range = max_range;
  range_msg.min_range = min_range;
  range_msg.header.frame_id = frame_id;
  range_msg.radiation_type = Range::INFRARED;

  // If handshake ends successfully, start reader thread and update timer.
  RCLCPP_INFO(this->get_logger(), "I²C communication ready");
}

/**
 * @brief Routine to initialize callback groups.
 */
void TerarangerNode::init_cgroups()
{
  // Topic subscriptions
  pose_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void TerarangerNode::init_publishers()
{
  // Range topic
  if (publish_range)
    range_pub_ = this->create_publisher<Range>(range_topic, 1);

  // Altitude topic
  altitude_pub_ = this->create_publisher<PoseWithCovarianceStamped>(altitude_topic, 1);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void TerarangerNode::init_subscribers()
{
  // Pose
  auto pose_sub_opt = rclcpp::SubscriptionOptions();
  pose_sub_opt.callback_group = pose_clbk_group_;
  pose_sub_ = this->create_subscription<PoseStamped>(
    pose_topic,
    1,
    std::bind(
      &TerarangerNode::pose_clbk,
      this,
      std::placeholders::_1),
    pose_sub_opt);
}

/**
 * @brief Routine to initialize timers.
 */
void TerarangerNode::init_timers()
{
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0/timer_frequency),
    std::bind(&TerarangerNode::timer_clbk, this));
}

/**
 * @brief Routine to initialize TF listeners.
 */
void TerarangerNode::init_tf_listeners()
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  while (1)
  {
    try {
      tf_map_odom = tf_buffer_->lookupTransform("/map", "/stanis/odom", tf2::TimePointZero);
      tf_laser_fmu = tf_buffer_->lookupTransform("/stanis/laser_link", "/stanis/fmu_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Are TFs being published? Trying again in 3 seconds...");
      rclcpp::sleep_for(3s);
    }
  }

  // Camera to base transformation
  get_isometry3d(tf_map_odom, iso_map_odom);

  // Laser to FMU transformation
  get_isometry3d(tf_laser_fmu, iso_laser_fmu);
}

} // namespace Teraranger
