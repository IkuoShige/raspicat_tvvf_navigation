#include "raspicat_tvvf_navigation/command_utils.hpp"
#include <cctype>

namespace raspicat_tvvf_navigation
{

namespace {

std::string trim_copy(const std::string &str)
{
  const auto first = str.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = str.find_last_not_of(" \t\n\r");
  return str.substr(first, last - first + 1);
}

}  // namespace

ParsedCommand parse_command(const std::string &command)
{
  const std::string trimmed = trim_copy(command);

  if (trimmed.empty()) {
    return ParsedCommand{ToleranceMode::LOOSE, ""};
  }

  const auto delimiter = trimmed.find_first_of(" \t");
  const std::string first_token = trimmed.substr(0, delimiter);
  const std::string rest =
    (delimiter == std::string::npos) ? "" : trim_copy(trimmed.substr(delimiter + 1));

  if (first_token == "strict") {
    return ParsedCommand{ToleranceMode::STRICT, rest};
  }
  if (first_token == "loose") {
    return ParsedCommand{ToleranceMode::LOOSE, rest};
  }

  // No tolerance modifier; keep command as-is with loose tolerance.
  return ParsedCommand{ToleranceMode::LOOSE, trimmed};
}

bool is_action_command(const std::string &command)
{
  const auto parsed = parse_command(command);
  return !parsed.action_command.empty();
}

}  // namespace raspicat_tvvf_navigation
