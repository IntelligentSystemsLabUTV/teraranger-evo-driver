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
  // Initialize parameters
  init_parameters();

  // Initialize topic publishers
  init_publishers();

  // Initialize TF listener
  init_tf_listeners();

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
  range_msg.max_range = range_max;
  range_msg.min_range = range_min;
  range_msg.header.frame_id = frame_map;
  range_msg.radiation_type = Range::INFRARED;

  // If handshake ends successfully, start reader thread and update timer.
  RCLCPP_INFO(this->get_logger(), "I²C communication ready");
}

/**
 * @brief Routine to initialize topic publishers.
 */
void TerarangerNode::init_publishers()
{
  // Range topic
  if (publish_range)
    range_pub_ = this->create_publisher<Range>(
      topic_range,
      DUAQoS::get_datum_qos());

  // Altitude topic
  if (publish_altitude)
    altitude_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
      topic_altitude,
      DUAQoS::get_datum_qos());
}

/**
 * @brief Routine to initialize timers.
 */
void TerarangerNode::init_timers()
{
  laser_timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0/timer_frequency),
    std::bind(
      &TerarangerNode::laser_timer_callback,
      this));
}

/**
 * @brief Routine to initialize TF listeners and their timer.
 */
void TerarangerNode::init_tf_listeners()
{
  // Initialize TF buffers and listeners
  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

} // namespace Teraranger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Teraranger::TerarangerNode)
