/**
 * Teraranger utilities.
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

#define NOOP ((void)0)

#include <teraranger/teraranger.hpp>

namespace Teraranger
{

/**
 * @brief Updates tf2 transforms.
 */
void TerarangerNode::tf_timer_callback()
{
  TransformStamped map_to_odom_{};
  TransformStamped laser_to_fmu_{};

  // Start listening
  // map -> odom
  try {
    map_to_odom_ = tf_buffer->lookupTransform(
      map_frame,
      odom_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    map_to_odom = map_to_odom;
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }

  // laser -> fmu
  try {
    laser_to_fmu_ = tf_buffer->lookupTransform(
      laser_frame,
      fmu_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    laser_to_fmu = laser_to_fmu_;
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }
}

/**
 * @brief Receives new messages from device, parses it and publishes data on topic
 */
void TerarangerNode::laser_timer_callback()
{
  // Send trigger and receive data
  uint8_t buf[1] = {0x00};
  if (this->swrite(buf, 1) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "swrite() failed");
    rclcpp::shutdown();
    return;
  }

  if (this->sread(buffer, BUFFER_SIZE) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "sread() failed");
    rclcpp::shutdown();
    return;
  }

  // Parse received data
  if (crc8(buffer, BUFFER_SIZE - 1) == buffer[BUFFER_SIZE - 1])
  {
    int16_t range = (buffer[0] << 8) | buffer[1];
    float final_range;

    switch (range)
    {
      case TOO_CLOSE_VALUE:
        final_range = -std::numeric_limits<float>::infinity();
        break;
      case OUT_OF_RANGE_VALUE:
        final_range = std::numeric_limits<float>::infinity();
        break;
      case INVALID_MEASURE:
        final_range = std::numeric_limits<float>::quiet_NaN();
        break;
      default:
        final_range = range * VALUE_TO_METER_FACTOR;
    }

    // Publish range msg if topic is enabled
    if (publish_range)
    {
      range_msg.header.stamp = this->get_clock()->now();
      range_msg.range = final_range;
      range_pub_->publish(range_msg);
    }

    PoseStamped last_drone_pose{};
    pose_mtx.lock();
    last_drone_pose = drone_pose;
    pose_mtx.unlock();

    Eigen::Quaterniond q_drone = Eigen::Quaterniond(last_drone_pose.pose.orientation.w,
                                                    last_drone_pose.pose.orientation.x,
                                                    last_drone_pose.pose.orientation.y,
                                                    last_drone_pose.pose.orientation.z);
    // roll = rpy[2], pitch = rpy[1], yaw = rpy[0]
    Eigen::Vector3d rpy = q_drone.toRotationMatrix().eulerAngles(2, 1, 0);

    Eigen::Vector3d p_drone = Eigen::Vector3d(last_drone_pose.pose.position.x,
                                              last_drone_pose.pose.position.y,
                                              last_drone_pose.pose.position.z);

    TransformStamped map_to_odom_{}, laser_to_fmu_{};
    tf_lock_.lock();
    laser_to_fmu_ = laser_to_fmu;
    map_to_odom_ = map_to_odom;
    tf_lock_.unlock();

    double altitude_tilted_map = final_range +
                                 fabs(laser_to_fmu_.transform.translation.z) +                     // z offset between laser and fmu
                                 fabs(laser_to_fmu_.transform.translation.x) * std::tan(rpy[1]);   // distance from the ground due to pitch
    double altitude_map = altitude_tilted_map * std::tan(rpy[2]) * std::tan(rpy[1]);
    double altitude_odom = altitude_map - fabs(map_to_odom_.transform.translation.z);

    PoseWithCovarianceStamped altitude_msg;

    // Header
    altitude_msg.header.set__stamp(this->get_clock()->now());
    altitude_msg.header.set__frame_id(odom_frame);

    // Position
    altitude_msg.pose.pose.position.set__z(altitude_odom);

    // Covariances
    if (fabs(altitude_odom - p_drone.z()) < delta_max)
      cov_vec[14] = cov_good;
    else
      cov_vec[14] = cov_bad;

    altitude_msg.pose.set__covariance(cov_vec);

    altitude_pub_->publish(altitude_msg);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "crc mismatch");
  }
}

} // namespace Teraranger
