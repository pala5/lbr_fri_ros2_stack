#ifndef LBR_FRI_ROS2__INTERFACES__DIGITAL_IO_COMMAND_HPP_
#define LBR_FRI_ROS2__INTERFACES__DIGITAL_IO_COMMAND_HPP_

#include <algorithm>

#include "lbr_fri_ros2/interfaces/io_command.hpp"

namespace lbr_fri_ros2 {
class DigitalIOCommandInterface : public IOCommandInterface {
protected:
  std::string LOGGER_NAME() const override { return "lbr_fri_ros2::DigitalIOCommandInterface"; }

public:
  DigitalIOCommandInterface() = delete;
  DigitalIOCommandInterface();

  void buffered_command_to_fri(fri_command_t_ref command) override;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__INTERFACES__DIGITAL_IO_COMMAND_HPP_
