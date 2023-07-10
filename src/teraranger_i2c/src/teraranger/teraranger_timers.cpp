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

    EulerPoseStamped last_drone_pose{};
    pose_mtx.lock();
    last_drone_pose = drone_pose;
    pose_mtx.unlock();

    double roll = last_drone_pose.roll;
    double pitch = last_drone_pose.pitch;

    double z = last_drone_pose.pose.position.z;

    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "z: " << z << std::endl;

    TransformStamped map_to_odom_{}, laser_to_fmu_{};
    tf_lock_.lock();
    laser_to_fmu_ = laser_to_fmu;
    map_to_odom_ = map_to_odom;
    tf_lock_.unlock();

    std::cout << "map_to_odom: " << map_to_odom_.transform.translation.x << " " << map_to_odom_.transform.translation.y << " " << map_to_odom_.transform.translation.z << std::endl;
    std::cout << "laser_to_fmu: " << laser_to_fmu_.transform.translation.x << " " << laser_to_fmu_.transform.translation.y << " " << laser_to_fmu_.transform.translation.z << std::endl;

    double altitude_tilted_map = final_range +
                                 fabs(laser_to_fmu_.transform.translation.z) +                     // z offset between laser and fmu
                                 fabs(laser_to_fmu_.transform.translation.x) * std::tan(pitch);   // distance from the ground due to pitch
    double altitude_map = altitude_tilted_map * std::tan(roll) * std::tan(pitch);
    double altitude_odom = altitude_map - fabs(map_to_odom_.transform.translation.z);

    std::cout << "altitude_tilted_map: " << altitude_tilted_map << std::endl;
    std::cout << "altitude_map: " << altitude_map << std::endl;
    std::cout << "altitude_odom: " << altitude_odom << std::endl;

    PoseWithCovarianceStamped altitude_msg;

    // Header
    altitude_msg.header.set__stamp(this->get_clock()->now());
    altitude_msg.header.set__frame_id(odom_frame);

    // Position
    altitude_msg.pose.pose.position.set__z(altitude_odom);

    // Covariances
    if (fabs(altitude_odom - z) < delta_max)
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
