#include "raspicat_tvvf_navigation/command_utils.hpp"

namespace raspicat_tvvf_navigation
{

bool is_action_command(const std::string &command)
{
  if (command.empty()) {
    return false;
  }
  if (command == "strict" || command == "loose") {
    return false;
  }
  return true;
}

}  // namespace raspicat_tvvf_navigation
