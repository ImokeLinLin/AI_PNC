#ifndef CORE_MODULES_REFERENCE_PATH_GENERATOR_INCLUDE_GOAL_SAMPLE_GENERATOR_H_
#define CORE_MODULES_REFERENCE_PATH_GENERATOR_INCLUDE_GOAL_SAMPLE_GENERATOR_H_

#include "core/framework/frame.h"
#include "core/modules/common/math/pnc_kdpath.h"
#include "motion_plan_common.pb.h"
#include "planning_proto_data.pb.h"
// #include "pnc_path_with_ppp_input.h"
namespace reference_path_generator {
struct GoalInfo {
  double relative_to_ego_s = 0.0;
  double relative_to_ref_l = 0.0;
  bool ego_in_reverse_road = false;
};

class GoalSampleGenerator {
  using RefPoints = ::google::protobuf::RepeatedPtrField<npp::pnc::RefPoint>;

 public:
  GoalSampleGenerator() = default;
  ~GoalSampleGenerator() = default;
  bool GenerateGoalSample(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
      const RefPoints &ref_points,
      const npp::planning_math::PncKDPath &target_lane,
      const double coarse_path_length, npp::framework::Frame *frame,
      std::vector<reference_path_generator::GoalInfo> &goal_info);
  bool GenerateGoalSampleForReverseLane(
      const npp::framework::Frame *frame,
      const double confidence_ego_in_reverse_lane,
      const double coarse_path_length,
      std::vector<reference_path_generator::GoalInfo> &goal_info) const;

 private:
  bool IsRoiAreaInIntersection(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
      const int roi_index1, const int roi_index2,
      const RefPoints &ref_points) const;
  double CalcRightRbProtectDistThresh(const RefPoints &ref_points,
                                      const int min_road_width_index) const;
  bool EgoInNoLBRoad(const RefPoints &RefPoints, const int roi_start_idx,
                     const int roi_end_idx) const;
  bool IsCountryRoad(
      const npp::framework::Frame *frame,
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
      const int roi_index2, const RefPoints &ref_points,
      double &country_road_start_offset) const;
  bool CalcRoiIndex(const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
                    const npp::planning_math::PncKDPath &target_lane,
                    const double ego_speed, const double coarse_path_length,
                    int &ego_index, int &first_roi_index,
                    int &second_roi_index) const;
  double CalcDistanceToRight(const double min_road_width) const;
  bool GoalSampleTrigger(
      const npp::framework::Frame *frame,
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
      const RefPoints &ref_points,
      const npp::planning_math::PncKDPath &target_lane, const int ego_index,
      const int first_roi_index, const int second_roi_index,
      const bool is_roi_in_intersection) const;

  bool IsValidBorderDist(const int min_left_border_dist_index,
                         const int min_road_width_index,
                         const double min_road_width,
                         const double min_left_border_dist,
                         const RefPoints &ref_points,
                         const bool is_roi_in_intersection,
                         const double min_left_border_dist_history) const;
  bool IsConsiderIntersection(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &input) const;
  bool FindCountryRoadFromAttributes(
      const std::vector<maf_landmark::AttributeOnPath> &attribute_on_path,
      double &country_road_start_offset, int &start_index) const;
  void GetCountryRoadEndOffset(
      const std::vector<maf_landmark::AttributeOnPath> &attribute_on_path,
      const int start_index, double &country_road_end_offset) const;

  const int kNormalFormway = 15; // 15 是普通道路
};
} // namespace reference_path_generator

#endif // CORE_MODULES_REFERENCE_PATH_GENERATOR_INCLUDE_GOAL_SAMPLE_GENERATOR_H_