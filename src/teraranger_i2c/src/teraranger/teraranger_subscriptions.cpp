/**
 * Converter topic subscription callbacks.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
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
 * @brief Logs drone's pose.
 *
 * @param msg Pose message to parse.
 */
void TerarangerNode::pose_clbk(const EulerPoseStamped::ConstSharedPtr msg)
{
  std::cout << "Received pose: " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << std::endl;
  pose_mtx.lock();
  drone_pose = *msg;
  pose_mtx.unlock();
}

} // namespace Teraranger
