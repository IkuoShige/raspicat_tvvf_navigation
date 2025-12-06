#include <gtest/gtest.h>
#include "raspicat_tvvf_navigation/command_utils.hpp"

using raspicat_tvvf_navigation::is_action_command;
using raspicat_tvvf_navigation::parse_command;
using raspicat_tvvf_navigation::ParsedCommand;
using raspicat_tvvf_navigation::ToleranceMode;

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

TEST(CommandUtilsTest, StrictPrefixIsRemovedForAction)
{
  EXPECT_TRUE(is_action_command("strict pause"));
  EXPECT_TRUE(is_action_command("strict wait:4"));
  EXPECT_FALSE(is_action_command("strict"));
}

TEST(CommandUtilsTest, ParseCommandHandlesTolerancePrefixes)
{
  ParsedCommand parsed = parse_command("strict pause");
  EXPECT_EQ(parsed.tolerance_mode, ToleranceMode::STRICT);
  EXPECT_EQ(parsed.action_command, "pause");

  parsed = parse_command("loose wait:2.5");
  EXPECT_EQ(parsed.tolerance_mode, ToleranceMode::LOOSE);
  EXPECT_EQ(parsed.action_command, "wait:2.5");

  parsed = parse_command("strict");
  EXPECT_EQ(parsed.tolerance_mode, ToleranceMode::STRICT);
  EXPECT_TRUE(parsed.action_command.empty());
}

TEST(CommandUtilsTest, ParseCommandTrimsWhitespace)
{
  ParsedCommand parsed = parse_command("  strict   wait_topic:/go   ");
  EXPECT_EQ(parsed.tolerance_mode, ToleranceMode::STRICT);
  EXPECT_EQ(parsed.action_command, "wait_topic:/go");

  parsed = parse_command("   pause  ");
  EXPECT_EQ(parsed.tolerance_mode, ToleranceMode::LOOSE);
  EXPECT_EQ(parsed.action_command, "pause");
}
