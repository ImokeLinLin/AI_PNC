#pragma once
#include <vector>

#include "core/modules/common/math/box2d.h"
#include "core/modules/common/math/pnc_kdpath.h"
#include "core/modules/path_planner/path_planner_preprocessor/pnc_path_with_ppp_input.h"
#include "core/proto/cartesian_road_boundaries.pb.h"
#include "core/proto/path_plan_preprocessor.pb.h"
namespace reference_path_generator {
class ReferencePathPreprocessor {
 public:
  ReferencePathPreprocessor() = default;
  ~ReferencePathPreprocessor() = default;

  void InitFcs(const path_planner::ProtoRefPoints &ref_points,
               const npp::pnc::RefLinesInfo &ref_lines_info,
               std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs);
  double CalCoarsePathLength(const path_planner::ProtoRefPoints &ref_points,
                             const double lat_path_length,
                             const npp::pnc::State &init_state,
                             const bool is_in_uturn);
  void Update(npp::pnc::path_plan_pre::PPPreprocessorInput &input,
              npp::pnc::path_plan_pre::PPPreprocessorOutput &output, const bool is_hpa);
  void FilterAndTransToProto(const npp::pnc::RoadBoundary &segments,
                             npp::pnc::RoadBoundary *res) const;

 private:
  void GenerateReferenceBBox(const double init_s, const double init_l,
                             const double coarse_path_length,
                             std::vector<npp::planning_math::Box2d> *res);
  double GenFinalLength(const npp::pnc::State &init_state,
                        const npp::pnc::TrajInfo *traj_info);
  void UpdateRefLinesInfo(npp::pnc::TrajInfo *traj_info);
  void UpdateDdpPathInfo(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
      npp::pnc::TrajInfo *traj_info);
  void SaveInitVelRatio(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &pppreprocessor_input,
      npp::pnc::path_plan_pre::PPPreprocessorOutput &output);
  void UpdateInputInitState(
      const double vel_ratio,
      npp::pnc::path_plan_pre::PPPreprocessorInput &pppreprocessor_input);
  bool OverlapWithRefPathBox(
      const npp::planning_math::Vec2d &pt,
      const std::vector<npp::planning_math::Box2d> &ref_box) const;

  std::vector<npp::planning_math::Box2d> ref_box_{};
  std::shared_ptr<path_planner::PncPathWithPPPInput> fcs_ = nullptr;
};
} // namespace reference_path_generator