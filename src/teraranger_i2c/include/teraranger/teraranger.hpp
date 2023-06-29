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

#ifndef TERARANGER_HPP
#define TERARANGER_HPP

#include <bitset>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <limits>
#include <mutex>

#include <dua_node/dua_node.hpp>

#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace rcl_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono;

#define AVAILABLE_CHECK \
  if (fd < 0)           \
    throw "Teraranger: uninitialized"

#define OUT_OF_RANGE_VALUE -1
#define TOO_CLOSE_VALUE     0
#define INVALID_MEASURE     1

#define VALUE_TO_METER_FACTOR 0.001
#define I2C_ADDR 0x31
#define BUFFER_SIZE 3

namespace Teraranger
{
const uint8_t crc_table[] = {0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23,
                             0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41,
                             0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf,
                             0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
                             0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc,
                             0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
                             0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20,
                             0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
                             0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74,
                             0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8,
                             0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6,
                             0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
                             0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 0x05, 0x02,
                             0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40, 0x47,
                             0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39,
                             0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
                             0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d,
                             0x84, 0x83, 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
                             0xfa, 0xfd, 0xf4, 0xf3};

class TerarangerNode : public DUANode::NodeBase
{
  public:
    TerarangerNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());
    ~TerarangerNode();

  private:
    /* Node initialization routines */
    void init_cgroups();
    void init_i2c();
    void init_parameters();
    void init_publishers();
    void init_subscribers();
    void init_tf_listeners();
    void init_timers();

    /* Timers callback groups */
    rclcpp::CallbackGroup::SharedPtr laser_timer_cgroup;
    rclcpp::CallbackGroup::SharedPtr tf_timer_cgroup;

    /* Timers */
    rclcpp::TimerBase::SharedPtr laser_timer;
    rclcpp::TimerBase::SharedPtr tf_timer;

    /* Timer callbacks */
    void laser_timer_callback();
    void tf_timer_callback();


    rclcpp::TimerBase::SharedPtr timer_;

    /* Topic publishers */
    rclcpp::Publisher<Range>::SharedPtr range_pub_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr altitude_pub_;

    /* Topic subscribers */
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;

    /* Callback groups */
    rclcpp::CallbackGroup::SharedPtr pose_clbk_group_;

    /* Callback functions */
    void pose_clbk(const PoseStamped::ConstSharedPtr msg);

    /* Parameters */
    std::string altitude_topic;
    double cov_good;
    double cov_bad;
    double delta_max;
    std::string link_namespace;
    std::string port;
    std::string pose_topic;
    bool publish_range;
    std::string range_topic;
    int64_t timer_frequency;

    /* Utility routines */
    int begin(const char *port);
    uint8_t crc8(uint8_t *p, uint8_t len);
    void end();
    void get_isometry3d(TransformStamped& tf_msg, Eigen::Isometry3d& iso);
    int sread(unsigned char *buff, ssize_t size);
    int swrite(unsigned char *buff, ssize_t size);

    /* Synchronization variables */
    std::mutex pose_mtx;
    std::mutex tf_lock_;

    /* Variables */
    int fd;
    uint8_t buffer[BUFFER_SIZE];

    Range range_msg;
    double field_of_view = 0.0349066;
    double max_range = 15.0;
    double min_range = 0.5;
    std::array<double, 36> cov_vec;

    PoseStamped::ConstSharedPtr drone_pose;

    /* TF */
    std::string map_frame, odom_frame, laser_frame, fmu_frame;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    TransformStamped map_to_odom{}, laser_to_fmu{};
};

} // namespace Teraranger

#endif // TERARANGER_HPP