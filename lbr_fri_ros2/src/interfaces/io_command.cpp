#include "lbr_fri_ros2/interfaces/io_command.hpp"

namespace lbr_fri_ros2 {

IOCommandInterface::IOCommandInterface(){};

void BaseCommandInterface::init_command() {
  // command_target_.boolean_io_value.fill(0);
  command_target_.digital_io_value.fill(0);
  // command_target_.analog_io_value.fill(0);
  command_ = command_target_;
}