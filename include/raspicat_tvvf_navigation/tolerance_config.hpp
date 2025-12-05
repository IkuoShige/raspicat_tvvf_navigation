#ifndef RASPICAT_TVVF_NAVIGATION__TOLERANCE_CONFIG_HPP_
#define RASPICAT_TVVF_NAVIGATION__TOLERANCE_CONFIG_HPP_

namespace raspicat_tvvf_navigation
{

struct ToleranceConfig
{
  double position_strict;
  double orientation_strict;
  double position_loose;
  double orientation_loose;
};

/**
 * @brief レガシーパラメータ（position_tolerance, orientation_tolerance）を考慮して
 *        厳格/緩和トレランスを解決する。
 *
 * ルール:
 *  - legacy_* が0より大きい場合、かつ strict/loose がデフォルト値のままなら上書きする。
 *  - それ以外は raw を優先する。
 */
ToleranceConfig resolve_tolerance_config(
  const ToleranceConfig &raw,
  const ToleranceConfig &defaults,
  double legacy_position_tolerance,
  double legacy_orientation_tolerance);

}  // namespace raspicat_tvvf_navigation

#endif  // RASPICAT_TVVF_NAVIGATION__TOLERANCE_CONFIG_HPP_
