#include <gtest/gtest.h>
#include "raspicat_tvvf_navigation/command_utils.hpp"

using raspicat_tvvf_navigation::is_action_command;

TEST(CommandUtilsTest, EmptyIsNotAction)
{
  EXPECT_FALSE(is_action_command(""));
}

TEST(CommandUtilsTest, LooseAndStrictAreNotAction)
{
  EXPECT_FALSE(is_action_command("loose"));
  EXPECT_FALSE(is_action_command("strict"));
}

TEST(CommandUtilsTest, WaitAndPauseAreAction)
{
  EXPECT_TRUE(is_action_command("wait:3"));
  EXPECT_TRUE(is_action_command("pause"));
  EXPECT_TRUE(is_action_command("stop"));
}
