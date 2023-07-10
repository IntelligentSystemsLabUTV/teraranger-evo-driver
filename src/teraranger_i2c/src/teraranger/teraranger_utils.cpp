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
 * @brief Routine to transform TransformStamped msg to Isometry3d structure.
 *
 * @param tf_msg PoseStamped message to be transformed.
 * @param iso Output Isometry3d.
 */
// void TerarangerNode::get_isometry3d(TransformStamped& tf_msg, Eigen::Isometry3d& iso)
// {
//   iso = Eigen::Isometry3d::Identity();
//   iso.rotate(Eigen::Quaterniond(tf_msg.transform.rotation.w,
//                                 tf_msg.transform.rotation.x,
//                                 tf_msg.transform.rotation.y,
//                                 tf_msg.transform.rotation.z));
//   iso.pretranslate(Eigen::Vector3d(tf_msg.transform.translation.x,
//                                     tf_msg.transform.translation.y,
//                                     tf_msg.transform.translation.z));
// }

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

} // namespace Teraranger
