#include "raspicat_tvvf_navigation/tolerance_config.hpp"
#include <algorithm>

namespace raspicat_tvvf_navigation
{

namespace {
bool is_positive(double v) { return v > 0.0; }
}  // namespace

ToleranceConfig resolve_tolerance_config(
  const ToleranceConfig &raw,
  const ToleranceConfig &defaults,
  double legacy_position_tolerance,
  double legacy_orientation_tolerance)
{
  ToleranceConfig resolved = raw;

  const bool legacy_pos = is_positive(legacy_position_tolerance);
  const bool legacy_ori = is_positive(legacy_orientation_tolerance);

  // 厳格トレランス: パラメータ未設定（デフォルト値のまま）のとき、レガシーで上書き
  if (legacy_pos && raw.position_strict == defaults.position_strict) {
    resolved.position_strict = legacy_position_tolerance;
  }
  if (legacy_ori && raw.orientation_strict == defaults.orientation_strict) {
    resolved.orientation_strict = legacy_orientation_tolerance;
  }

  // 緩和トレランス: パラメータ未設定（デフォルト値のまま）のとき、レガシーで上書き
  if (legacy_pos && raw.position_loose == defaults.position_loose) {
    resolved.position_loose = legacy_position_tolerance;
  }
  if (legacy_ori && raw.orientation_loose == defaults.orientation_loose) {
    resolved.orientation_loose = legacy_orientation_tolerance;
  }

  // 非負にクリップ（異常値防止）
  resolved.position_strict = std::max(0.0, resolved.position_strict);
  resolved.orientation_strict = std::max(0.0, resolved.orientation_strict);
  resolved.position_loose = std::max(0.0, resolved.position_loose);
  resolved.orientation_loose = std::max(0.0, resolved.orientation_loose);

  return resolved;
}

}  // namespace raspicat_tvvf_navigation
