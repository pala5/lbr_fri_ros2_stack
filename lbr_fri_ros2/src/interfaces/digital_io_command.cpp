#include "lbr_fri_ros2/interfaces/digital_io_command.hpp"

namespace lbr_fri_ros2 {
DigitalIOCommandInterface::DigitalIOCommandInterface()
    : IOCommandInterface() {}

void DigitalIOCommandInterface::buffered_command_to_fri(fri_command_t_ref command) {
  // check if command_ is valid, if not restore
  if (std::any_of(command_target_.digital_io_value.cbegin(), command_target_.digital_io_value.cend(),
                  [](const uint64 &v) { return std::isnan(v); })) {
    this->init_command();
  }

  // write joint position to output
  command.setDigitalIOValue(command_.digital_io_value.data());
}
} // namespace lbr_fri_ros2
