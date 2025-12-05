#ifndef RASPICAT_TVVF_NAVIGATION__COMMAND_UTILS_HPP_
#define RASPICAT_TVVF_NAVIGATION__COMMAND_UTILS_HPP_

#include <string>

namespace raspicat_tvvf_navigation
{

// trueを返すコマンドのみ「アクション」として実行する。
// 現状、strict/loose/空文字はアクション扱いしない。
bool is_action_command(const std::string &command);

}  // namespace raspicat_tvvf_navigation

#endif  // RASPICAT_TVVF_NAVIGATION__COMMAND_UTILS_HPP_
