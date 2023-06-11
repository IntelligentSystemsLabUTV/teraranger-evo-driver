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

#include <teraranger/teraranger.hpp>

namespace Teraranger
{

/**
 * @brief Starts a serial connection.
 *
 * @param port String with port address.
 * @param baudrate
 * @return 0 if ok, -1 if error.
 */
int TerarangerNode::begin(const char *port)
{
  fd = open(port, O_RDWR);
  if (fd == -1) {
    perror("Unable to open I²C device");
    return -1;
  }

  if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
    perror("ioctl error");
    return -1;
  }

  return 0;
}

/**
 * @brief Ends an I²C connection.
 *
 */
void TerarangerNode::end()
{
  if (fd > 0)
    close(fd);
}

/**
 * @brief Reads bytes from serial connection.
 *
 * @param buff Buffer to fill with data retrieved.
 * @param size Size of the incoming data.
 * @return Number of bytes read if ok, -1 if error.
 */
int TerarangerNode::sread(unsigned char *buff, ssize_t size)
{
  AVAILABLE_CHECK;
  ssize_t bread = 0;
  while (bread < size)
  {
    int n = read(fd, buff + bread, size - bread);
    if (n < 0)
    {
      int errsv = errno;
      if (errsv == EINTR)
      {
        continue;
      }
      else
      {
        return -1;
      }
    }
    bread += n;
  }
  return bread;
}

/**
 * @brief Sends bytes over serial connection.
 *
 * @param buff Bytes to send.
 * @param size Size of the buffer.
 * @return Number of bytes sent if ok, -1 if error.
 */
int TerarangerNode::swrite(unsigned char *buff, ssize_t size)
{
  AVAILABLE_CHECK;
  ssize_t bwritten = 0;
  while (bwritten < size)
  {
    int n = write(fd, buff + bwritten, size - bwritten);
    if (n < 0)
    {
      int errsv = errno;
      if (errsv == EINTR)
      {
        continue;
      }
      else
      {
        return -1;
      }
    }
    bwritten += n;
  }
  return bwritten;
}

/**
 * @brief Routine to initialize a spinlock.
 *
 * @param lock Pointer to the lock to initialize.
 *
 * @throws RuntimeError
 */
void TerarangerNode::create_spinlock(pthread_spinlock_t * lock)
{
  if (pthread_spin_init(lock, PTHREAD_PROCESS_PRIVATE)) {
    throw std::runtime_error("Failed to initialize spinlock");
  }
}

/**
 * @brief Routine to transform TransformStamped msg to Isometry3d structure.
 *
 * @param tf_msg PoseStamped message to be transformed.
 * @param iso Output Isometry3d.
 */
void TerarangerNode::get_isometry3d(TransformStamped& tf_msg, Eigen::Isometry3d& iso)
  {
    iso = Eigen::Isometry3d::Identity();
    iso.rotate(Eigen::Quaterniond(tf_msg.transform.rotation.w,
                                  tf_msg.transform.rotation.x,
                                  tf_msg.transform.rotation.y,
                                  tf_msg.transform.rotation.z));
    iso.pretranslate(Eigen::Vector3d(tf_msg.transform.translation.x,
                                     tf_msg.transform.translation.y,
                                     tf_msg.transform.translation.z));
  }

/**
 * @brief Receives new messages from device, parses it and publishes data on topic
 */
void TerarangerNode::timer_clbk()
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

    PoseStamped::ConstSharedPtr last_drone_pose;
    pose_mtx.lock();
    last_drone_pose = drone_pose;
    pose_mtx.unlock();

    Eigen::Quaterniond q_drone = Eigen::Quaterniond(last_drone_pose->pose.orientation.w,
                                                    last_drone_pose->pose.orientation.x,
                                                    last_drone_pose->pose.orientation.y,
                                                    last_drone_pose->pose.orientation.z);
    // roll = rpy[2], pitch = rpy[1], yaw = rpy[0]
    Eigen::Vector3d rpy = q_drone.toRotationMatrix().eulerAngles(2, 1, 0);

    Eigen::Vector3d p_drone = Eigen::Vector3d(last_drone_pose->pose.position.x,
                                              last_drone_pose->pose.position.y,
                                              last_drone_pose->pose.position.z);

    double altitude_tilted_map = final_range + 
                                 fabs(tf_laser_fmu.transform.translation.z) +                     // z offset between laser and fmu
                                 fabs(tf_laser_fmu.transform.translation.x) * std::tan(rpy[1]);   // distance from the ground due to pitch
    double altitude_map = altitude_tilted_map * std::tan(rpy[2]) * std::tan(rpy[1]);
    double altitude_odom = altitude_map - fabs(tf_map_odom.transform.translation.z);

    PoseWithCovarianceStamped altitude_msg;

    // Header
    altitude_msg.header.set__stamp(this->get_clock()->now());
    altitude_msg.header.set__frame_id(std::string("/odom"));

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

/**
 * @brief Computes current message CRC
 */
uint8_t TerarangerNode::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

/**
 * @brief Validates update of the altitude_topic parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_altitude_topic(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Teraranger::validate_altitude_topic: Altitude topic name cannot be empty");
    return false;
  }
  altitude_topic = p.as_string();
  return true;
}

/**
 * @brief Validates update of the cov_good parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_cov_good(const rclcpp::Parameter & p)
{
  cov_good = p.as_double();
  return true;
}

/**
 * @brief Validates update of the cov_bad parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_cov_bad(const rclcpp::Parameter & p)
{
  cov_bad = p.as_double();
  return true;
}

/**
 * @brief Validates update of the delta_max parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_delta_max(const rclcpp::Parameter & p)
{
  delta_max = p.as_double();
  return true;
}

/**
 * @brief Validates update of the port parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_port(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Teraranger::port: Port name cannot be empty");
    return false;
  }
  port = p.as_string();
  return true;
}

/**
 * @brief Validates update of the port parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_pose_topic(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Teraranger::pose_topic: Pose topic name cannot be empty");
    return false;
  }
  pose_topic = p.as_string();
  return true;
}

/**
 * @brief Validate update of the publish_range parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_publish_range(const rclcpp::Parameter & p)
{
  publish_range = p.as_bool();
  return true;
}

/**
 * @brief Validates update of the range_topic parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_range_topic(const rclcpp::Parameter & p)
{
  if (p.as_string().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Teraranger::range_topic: Range topic name cannot be empty");
    return false;
  }
  range_topic = p.as_string();
  return true;
}

/**
 * @brief Validate update of the timer_frequency parameter.
 *
 * @param p Parameter to be validated.
 * @return true if parameter is valid, false otherwise.
 */
bool TerarangerNode::validate_timer_frequency(const rclcpp::Parameter & p)
{
  timer_frequency = p.as_int();
  return true;
}

} // namespace Teraranger
