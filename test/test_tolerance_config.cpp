#include <gtest/gtest.h>
#include "raspicat_tvvf_navigation/tolerance_config.hpp"

using raspicat_tvvf_navigation::ToleranceConfig;
using raspicat_tvvf_navigation::resolve_tolerance_config;

namespace {
const ToleranceConfig kDefaults{0.3, 0.3, 0.5, 3.14};
}

TEST(ToleranceConfigTest, UsesLegacyWhenNewParamsAreDefault)
{
  ToleranceConfig raw = kDefaults;
  auto resolved = resolve_tolerance_config(raw, kDefaults, 0.8, 1.0);
  EXPECT_DOUBLE_EQ(resolved.position_strict, 0.8);
  EXPECT_DOUBLE_EQ(resolved.position_loose, 0.8);
  EXPECT_DOUBLE_EQ(resolved.orientation_strict, 1.0);
  EXPECT_DOUBLE_EQ(resolved.orientation_loose, 1.0);
}

TEST(ToleranceConfigTest, KeepsNewParamsWhenOverridden)
{
  ToleranceConfig raw{0.25, 0.4, 0.6, 2.5};
  auto resolved = resolve_tolerance_config(raw, kDefaults, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(resolved.position_strict, 0.25);
  EXPECT_DOUBLE_EQ(resolved.position_loose, 0.6);
  EXPECT_DOUBLE_EQ(resolved.orientation_strict, 0.4);
  EXPECT_DOUBLE_EQ(resolved.orientation_loose, 2.5);
}

TEST(ToleranceConfigTest, NegativeLegacyIgnored)
{
  ToleranceConfig raw = kDefaults;
  auto resolved = resolve_tolerance_config(raw, kDefaults, -1.0, -1.0);
  EXPECT_DOUBLE_EQ(resolved.position_strict, kDefaults.position_strict);
  EXPECT_DOUBLE_EQ(resolved.position_loose, kDefaults.position_loose);
  EXPECT_DOUBLE_EQ(resolved.orientation_strict, kDefaults.orientation_strict);
  EXPECT_DOUBLE_EQ(resolved.orientation_loose, kDefaults.orientation_loose);
}
