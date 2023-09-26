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
 * @brief Receives new messages from device, parses it and publishes data on topic
 */
void TerarangerNode::laser_timer_callback()
{
  if (reinit)
  {
    // Start serial interface and perform handshake
    if (begin(port.c_str()) == 0) 
    {
      reinit = false;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot open I²C port");

      // Wait before trying again
      std::this_thread::sleep_for(std::chrono::seconds(1));
      return;
    }
  }

  // Send trigger and receive data
  uint8_t buf[1] = {0x00};
  if (this->swrite(buf, 1) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "swrite() failed, trying to reconnect...");
    end();
    reinit = true;
    return;
  }

  if (this->sread(buffer, BUFFER_SIZE) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "swrite() failed, trying to reconnect...");
    end();
    init_i2c();
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

    if (publish_altitude)
    {
      // Get latest fixed_frame -> base_link transform
      TransformStamped odom_to_base{};
      rclcpp::Time tf_time = this->get_clock()->now();
      while (true) {
        try {
          odom_to_base = tf_buffer->lookupTransform(
            frame_odom,
            frame_base,
            tf_time,
            tf2::durationFromSec(0.1));
          break;
        } catch (const tf2::ExtrapolationException & e) {
          // Just get the latest
          tf_time = rclcpp::Time{};
        } catch (const tf2::TransformException & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "ObstacleDetectionNode::target_array_callback: TF exception: %s",
            e.what());
          return;
        }
      }
      Eigen::Isometry3d iso_odom_base = tf2::transformToEigen(odom_to_base).cast<double>();

      // Get quaternion and rpy angles from fixed_to_base
      Eigen::Quaterniond q_drone = Eigen::Quaterniond(iso_odom_base.rotation());

      // roll = rpy[2], pitch = rpy[1], yaw = rpy[0]
      Eigen::Vector3d rpy = q_drone.toRotationMatrix().eulerAngles(2, 1, 0);

      // Get position from fixed_to_base
      Eigen::Vector3d p_drone = Eigen::Vector3d(iso_odom_base.translation());


      // Get latest TFs
      TransformStamped map_to_odom{}, laser_to_fmu{};
      // map -> odom
      tf_time = this->get_clock()->now();
      try
      {
        map_to_odom = tf_buffer->lookupTransform(
          frame_map,
          frame_odom,
          tf2::TimePointZero,
          tf2::durationFromSec(1.0));
      }
      catch (const tf2::ExtrapolationException & e)
      {
        // Just get the latest
        tf_time = rclcpp::Time{};
      }
      catch (const tf2::TransformException & e)
      {
        RCLCPP_ERROR(this->get_logger(),
          "TerarangerNode::tf_timer_callback: TF exception: %s",
          e.what());
        return;
      }

      // laser -> fmu
      tf_time = this->get_clock()->now();
      try
      {
        laser_to_fmu = tf_buffer->lookupTransform(
          frame_laser,
          frame_fmu,
          tf2::TimePointZero,
          tf2::durationFromSec(1.0));
      }
      catch (const tf2::ExtrapolationException & e)
      {
        // Just get the latest
        tf_time = rclcpp::Time{};
      }
      catch (const tf2::TransformException & e)
      {
        RCLCPP_ERROR(this->get_logger(),
          "TerarangerNode::tf_timer_callback: TF exception: %s",
          e.what());
        return;
      }

      // Compute height
      double x_laser = laser_to_fmu.transform.translation.z;
      double z_laser = laser_to_fmu.transform.translation.x;

      double altitude_tilted_map = final_range +
                                  fabs(z_laser) +                      // z offset between laser and fmu
                                  fabs(x_laser) * std::tan(rpy[1]);    // distance from the ground due to pitch
      double altitude_map = altitude_tilted_map * std::cos(rpy[2]) * std::cos(rpy[1]);
      double altitude_odom = altitude_map - fabs(map_to_odom.transform.translation.z);

      PoseWithCovarianceStamped altitude_msg;

      // Header
      altitude_msg.header.set__stamp(this->get_clock()->now());
      altitude_msg.header.set__frame_id(frame_odom);

      // Position
      altitude_msg.pose.pose.position.set__z(altitude_odom);

      /////////////////////////////////////////////////
      // TODO
      // Covariances
      if (fabs(altitude_odom - p_drone[2]) < delta_max)
        cov_vec[14] = cov_good;
      else
        cov_vec[14] = cov_bad;

      altitude_msg.pose.set__covariance(cov_vec);
      /////////////////////////////////////////////////

      altitude_pub_->publish(altitude_msg);
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "crc mismatch");
  }
}

} // namespace Teraranger
