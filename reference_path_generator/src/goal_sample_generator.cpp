#include "goal_sample_generator.h"
#include "core/modules/common/ego_prediction_debug_logger.h"
#include "core/modules/common/utils/pose2d_utils.hpp"
namespace reference_path_generator {
bool GoalSampleGenerator::EgoInNoLBRoad(const RefPoints &RefPoints,
                                        const int roi_start_idx,
                                        const int roi_end_idx) const {
  constexpr double kRoadWidth = 4.5;
  constexpr double kRBLBDiff = 1.0;
  for (int i = roi_start_idx; i <= roi_end_idx; ++i) {
    const auto &p = RefPoints[i];
    if (p.s() < 0) {
      continue;
    }
    const auto &p_lane_bound = p.border().lane_bound();
    const auto &p_road_bound = p.border().road_bound();
    const bool curr_no_lb =
        p_lane_bound.lower() >
            std::fmin(p_road_bound.lower() - kRBLBDiff, kRoadWidth) &&
        p_lane_bound.upper() >
            std::fmin(p_road_bound.upper() - kRBLBDiff, kRoadWidth);
    const double curr_lane_width =
        std::min({p_road_bound.lower(), p_lane_bound.lower(), kRoadWidth}) +
        std::min({p_road_bound.upper(), p_lane_bound.upper(), kRoadWidth});
    const bool curr_in_wide = curr_lane_width > kRoadWidth;
    if (!curr_no_lb || !curr_in_wide) {
      MDEBUG_JSON_BEGIN_DICT(is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(rb_check_i, i, is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(curr_lane_width, curr_lane_width, is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(curr_no_lb, curr_no_lb, is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(distance_to_left_lane_border, p_lane_bound.upper(),
                           is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(distance_to_left_road_border, p_road_bound.upper(),
                           is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(distance_to_right_lane_border, p_lane_bound.lower(),
                           is_not_wide_lane)
      MDEBUG_JSON_ADD_ITEM(distance_to_right_road_border, p_road_bound.lower(),
                           is_not_wide_lane)
      MDEBUG_JSON_END_DICT(is_not_wide_lane)
      return false;
    }
  }
  return true;
}

bool GoalSampleGenerator::IsCountryRoad(
    const npp::framework::Frame *frame,
    const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
    const int roi_index2, const RefPoints &ref_points,
    double &country_road_start_offset) const {
  const auto &landmark_navi = frame->world_model().get_landmark_navi();
  if (landmark_navi == nullptr) {
    return false;
  }
  // formway list
  const std::vector<maf_landmark::AttributeOnPath> &attribute_on_path =
      landmark_navi->attribute_on_path;
  if (attribute_on_path.empty()) {
    return false;
  }
  int start_index = 0;
  if (!FindCountryRoadFromAttributes(attribute_on_path,
                                     country_road_start_offset, start_index)) {
    return false;
  }
  double country_road_end_offset = country_road_start_offset;
  GetCountryRoadEndOffset(attribute_on_path, start_index,
                          country_road_end_offset);
  MDEBUG_JSON_ADD_ITEM(country_road_start_offset, country_road_start_offset, 0)
  MDEBUG_JSON_ADD_ITEM(country_road_end_offset, country_road_end_offset, 0)
  if (country_road_end_offset < 0.0) {
    return false;
  }
  const double second_roi_offset =
      ref_points[roi_index2].s() - ppp_input.init_state().frenet_state().s();
  const bool is_roi_in_country_road =
      country_road_start_offset < second_roi_offset;
  MDEBUG_JSON_ADD_ITEM(second_roi_offset, second_roi_offset, 0)
  if (!is_roi_in_country_road) {
    return false;
  }
  if (IsConsiderIntersection(ppp_input)) {
    const double kRoiDistAfterIntersection = 10.0;
    const double dist_exit_intersection =
        ppp_input.intersection_info().dist_to_stopline() +
        ppp_input.intersection_info().intersection_length() +
        kRoiDistAfterIntersection;
    MDEBUG_JSON_ADD_ITEM(exit_intersection_offset, dist_exit_intersection, 0)
    if (dist_exit_intersection > country_road_end_offset) {
      return false;
    }
  }
  return true;
}

bool GoalSampleGenerator::FindCountryRoadFromAttributes(
    const std::vector<maf_landmark::AttributeOnPath> &attribute_on_path,
    double &country_road_start_offset, int &start_index) const {
  const int attribute_size = attribute_on_path.size();
  for (int i = 0; i < attribute_size; ++i) {
    const auto &it = attribute_on_path[i];
    if (!(it.available & maf_landmark::AttributeOnPath::OFFSET_VALID)) {
      continue;
    }
    std::string err_info{};
    mjson::Json parsed_reserved = mjson::Json::parse(it.reserved, err_info);
    if (parsed_reserved.has_key("formway") &&
        parsed_reserved.has_key("length") &&
        parsed_reserved["formway"].int_value() == kNormalFormway) {
      country_road_start_offset = it.start_offset;
      start_index = i;
      return true;
    }
  }
  return false;
}

void GoalSampleGenerator::GetCountryRoadEndOffset(
    const std::vector<maf_landmark::AttributeOnPath> &attribute_on_path,
    const int start_index, double &country_road_end_offset) const {
  const int attribute_size = attribute_on_path.size();
  for (int i = start_index; i < attribute_size; ++i) {
    const auto &it = attribute_on_path[i];
    if (!(it.available & maf_landmark::AttributeOnPath::OFFSET_VALID)) {
      break;
    }
    std::string err_info{};
    mjson::Json parsed_reserved = mjson::Json::parse(it.reserved, err_info);
    if (parsed_reserved.has_key("formway") &&
        parsed_reserved.has_key("length") &&
        parsed_reserved["formway"].int_value() == kNormalFormway) {
      country_road_end_offset += parsed_reserved["length"].number_value();
    } else {
      break;
    }
  }
}

bool GoalSampleGenerator::GenerateGoalSample(
    const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
    const RefPoints &ref_points,
    const npp::planning_math::PncKDPath &target_lane,
    const double coarse_path_length, npp::framework::Frame *frame,
    std::vector<reference_path_generator::GoalInfo> &goal_info) {
  if (frame == nullptr) {
    return false;
  }
  const int kInvalidIndex = -1;
  int ego_index = kInvalidIndex;
  int first_roi_index = kInvalidIndex;
  int second_roi_index = kInvalidIndex;
  const double ego_speed =
      frame->ego_prediction_context().ego_state()->velocity();
  // 如果遇到路口，roi会延伸到路口后
  if (!CalcRoiIndex(ppp_input, target_lane, ego_speed, coarse_path_length,
                    ego_index, first_roi_index, second_roi_index)) {
    frame->mutable_ego_prediction_context()->set_min_left_border_dist_history(
        0.0);
    return false;
  }
  const bool is_roi_in_intersection = IsRoiAreaInIntersection(
      ppp_input, first_roi_index, second_roi_index, ref_points);
  MDEBUG_JSON_ADD_ITEM(is_roi_in_intersection, is_roi_in_intersection, 0)
  const double min_left_border_dist_history =
      frame->ego_prediction_context().get_min_left_border_dist_history();
  MDEBUG_JSON_ADD_ITEM(min_left_border_dist_history,
                       min_left_border_dist_history, 0)
  if (!is_roi_in_intersection) {
    frame->mutable_ego_prediction_context()->set_min_left_border_dist_history(
        0.0);
  }
  if (!GoalSampleTrigger(frame, ppp_input, ref_points, target_lane, ego_index,
                         first_roi_index, second_roi_index,
                         is_roi_in_intersection)) {
    return false;
  }

  double min_left_border_dist = std::numeric_limits<double>::max();
  double min_road_width = std::numeric_limits<double>::max();
  int min_left_border_dist_index = -1;
  int min_road_width_index = -1;
  const double kInvalidRoadWidth = 99.0;
  for (int i = first_roi_index; i < second_roi_index; ++i) {
    const auto &point = ref_points.at(i);
    const double left_border_dist =
        std::min(point.border().road_bound().upper(),
                 point.border().lane_bound().upper());
    const double right_border_dist =
        std::min(point.border().road_bound().lower(),
                 point.border().lane_bound().lower());
    const double road_width = left_border_dist + right_border_dist;
    if (left_border_dist > kInvalidRoadWidth) {
      continue;
    }
    if (left_border_dist < min_left_border_dist) {
      min_left_border_dist = left_border_dist;
      min_left_border_dist_index = i;
    }
    if (road_width < min_road_width) {
      min_road_width = road_width;
      min_road_width_index = i;
    }
  }
  if (!IsValidBorderDist(min_left_border_dist_index, min_road_width_index,
                         min_road_width, min_left_border_dist, ref_points,
                         is_roi_in_intersection,
                         min_left_border_dist_history)) {
    return false;
  }

  const double right_rb_protect_dist_thresh =
      CalcRightRbProtectDistThresh(ref_points, min_road_width_index);
  const double kHalf = 0.5;
  const double to_right_diff_distance = CalcDistanceToRight(min_road_width);
  MDEBUG_JSON_ADD_ITEM(to_right_diff_distance, to_right_diff_distance, 0)
  const double target_l =
      min_left_border_dist -
      std::min(std::max((min_road_width * kHalf + to_right_diff_distance),
                min_left_border_dist),
                (min_road_width - right_rb_protect_dist_thresh));
  const double goal1_relative_s_to_ego =
      ref_points.at(min_left_border_dist_index).s() -
      ppp_input.init_state().frenet_state().s();
  const double kMaxTargetL = 2.5;
  const double kMaxTargetLInIntersection = 0.5;
  const double max_target_l =
      is_roi_in_intersection ? kMaxTargetLInIntersection : kMaxTargetL;
  const double kGain = 0.07;
  const double max_l =
      npp::clip(kGain * goal1_relative_s_to_ego, max_target_l, 0.0);
  const double final_target_l = npp::clip(target_l, max_l, -max_l);
  if (!is_roi_in_intersection) {
    frame->mutable_ego_prediction_context()->set_min_left_border_dist_history(
        min_left_border_dist - final_target_l);
  }
  goal_info.clear();
  goal_info.push_back({goal1_relative_s_to_ego, final_target_l, false});
  goal_info.push_back({coarse_path_length, 0.0, false});
  MDEBUG_JSON_BEGIN_DICT(goal_info)
  MDEBUG_JSON_ADD_ITEM(min_road_width, min_road_width, goal_info)
  MDEBUG_JSON_ADD_ITEM(min_left_border_dist, min_left_border_dist, goal_info)
  MDEBUG_JSON_ADD_ITEM(right_rb_protect_dist_thresh,
                       right_rb_protect_dist_thresh, goal_info)
  MDEBUG_JSON_ADD_ITEM(ego_s, ppp_input.init_state().frenet_state().s(),
                       goal_info)
  MDEBUG_JSON_ADD_ITEM(goal1_l, target_l, goal_info)
  MDEBUG_JSON_ADD_ITEM(goal1_relative_s, goal1_relative_s_to_ego, goal_info)
  MDEBUG_JSON_ADD_ITEM(goal2_relative_s, coarse_path_length, goal_info)
  MDEBUG_JSON_END_DICT(goal_info)
  return true;
}

bool GoalSampleGenerator::IsValidBorderDist(
    const int min_left_border_dist_index, const int min_road_width_index,
    const double min_road_width, const double min_left_border_dist,
    const RefPoints &ref_points, const bool is_roi_in_intersection,
    const double min_left_border_dist_history) const {
  const double kInvalidRoadWidth = 99.0;
  if ((min_left_border_dist_index < 0) || (min_road_width_index < 0) ||
      (min_road_width > kInvalidRoadWidth)) {
    return false;
  }
  const double kLeftBorderIgnoreDist = 10.0;
  MDEBUG_JSON_ADD_ITEM(min_left_border_dist, min_left_border_dist, 0)
  if (min_left_border_dist > kLeftBorderIgnoreDist) {
    return false;
  }
  const double kLeftBorderIgnoreProtectDist = 0.5;
  if (is_roi_in_intersection &&
      min_left_border_dist >
          min_left_border_dist_history - kLeftBorderIgnoreProtectDist) {
    return false;
  }
  const double min_right_border_dist = std::min(
      ref_points.at(min_road_width_index).border().lane_bound().lower(),
      ref_points.at(min_road_width_index).border().road_bound().lower());
  const double kRightBorderIgnoreDist = 19.99;
  MDEBUG_JSON_ADD_ITEM(min_right_border_dist, min_right_border_dist, 0)
  if (min_right_border_dist > kRightBorderIgnoreDist) {
    return false;
  }
  return true;
}

bool GoalSampleGenerator::GoalSampleTrigger(
    const npp::framework::Frame *frame,
    const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
    const RefPoints &ref_points,
    const npp::planning_math::PncKDPath &target_lane, const int ego_index,
    const int first_roi_index, const int second_roi_index,
    const bool is_roi_in_intersection) const {
  const bool is_ddld = npp::planner::ConfigurationContext::Instance()
                           .planner_config()
                           .is_ddld_navi;
  if (!is_ddld) {
    return false;
  }
  double country_road_start_offset = std::numeric_limits<double>::max();
  const bool is_country_road =
      IsCountryRoad(frame, ppp_input, second_roi_index, ref_points,
                    country_road_start_offset);
  MDEBUG_JSON_ADD_ITEM(is_country_road, is_country_road, 0)
  if (is_country_road) {
    int enter_country_road_index =
        target_lane
            .GetIndexFromS(ppp_input.init_state().frenet_state().s() +
                           country_road_start_offset)
            .id;
    enter_country_road_index =
        npp::clip(enter_country_road_index, second_roi_index, ego_index);
    const bool is_none_lb_super_wide_lane =
        EgoInNoLBRoad(ref_points, enter_country_road_index, second_roi_index);
    MDEBUG_JSON_ADD_ITEM(is_none_lb_in_country_road,
                         is_none_lb_super_wide_lane, 0)
    return is_none_lb_super_wide_lane;
  } else {
    const auto &is_ego_in_no_lb_road = frame->session()
                                           ->ego_prediction_context()
                                           .virtual_lane_manager()
                                           .GetEgoInNoLBRoad();
    MDEBUG_JSON_ADD_ITEM(is_ego_in_no_lb_road, is_ego_in_no_lb_road, 0)
    if ((!is_ego_in_no_lb_road) || is_roi_in_intersection) {
      return false;
    }
  }
  return true;
}

double GoalSampleGenerator::CalcDistanceToRight(
    const double min_road_width) const {
  const double kSlope = 0.1;
  const double kMinRoadWidth = 4.5;
  const double kMinToRightDiffDistance = 1.5;
  const double kMaxToRightDiffDistance = 2.0;
  return npp::clip(
      (min_road_width - kMinRoadWidth) * kSlope + kMinToRightDiffDistance,
      kMaxToRightDiffDistance, kMinToRightDiffDistance);
}

bool GoalSampleGenerator::IsRoiAreaInIntersection(
    const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
    const int roi_index1, const int roi_index2,
    const RefPoints &ref_points) const {
  if (IsConsiderIntersection(ppp_input)) {
    return true;
  }
  const double intersection_start_s =
      ppp_input.intersection_info().dist_to_stopline() +
      ppp_input.init_state().frenet_state().s();
  const double intersection_end_s =
      intersection_start_s +
      ppp_input.intersection_info().intersection_length();

  for (int i = roi_index1; i < roi_index2; ++i) {
    const auto &point = ref_points.at(i);
    if ((point.s() < intersection_end_s) &&
        (point.s() > intersection_start_s)) {
      MDEBUG_JSON_BEGIN_DICT(RoiAreaIsInIntersection)
      MDEBUG_JSON_ADD_ITEM(intersection_start_s, intersection_start_s,
                           RoiAreaIsInIntersection)
      MDEBUG_JSON_ADD_ITEM(intersection_end_s, intersection_end_s,
                           RoiAreaIsInIntersection)
      MDEBUG_JSON_END_DICT(RoiAreaIsInIntersection)
      return true;
    }
  }
  return false;
}

double GoalSampleGenerator::CalcRightRbProtectDistThresh(
    const RefPoints &ref_points, const int min_road_width_index) const {
  const double right_lane_dist =
      ref_points.at(min_road_width_index).border().lane_bound().lower();
  const double right_road_dist =
      ref_points.at(min_road_width_index).border().road_bound().lower();
  const double kRightRbProtectDist = 1.8;
  const double kRightLbProtectDist = 1.3;
  if (right_lane_dist <
      (right_road_dist - (kRightRbProtectDist - kRightLbProtectDist))) {
    return kRightLbProtectDist;
  } else {
    return kRightRbProtectDist -
           std::max((right_road_dist - right_lane_dist), 0.0);
  }
}

bool GoalSampleGenerator::CalcRoiIndex(
    const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
    const npp::planning_math::PncKDPath &target_lane, const double ego_speed,
    const double coarse_path_length, int &ego_index, int &first_roi_index,
    int &second_roi_index) const {
  const double kFirstRoiTime = 2.0;
  const double kSecondRoiTime = 4.0;
  const double kFirstRoiDist = 10.0;
  const double kSecondRoiDist = 40.0;
  const double ego_s = input.init_state().frenet_state().s();
  ego_index = target_lane.GetIndexFromS(ego_s).id;
  const double first_goal_s_dist =
      std::max(ego_speed * kFirstRoiTime, kFirstRoiDist);
  const double kRoiDistAfterIntersection = 10.0;
  const double dist_exit_intersection =
        IsConsiderIntersection(input)
            ? input.intersection_info().dist_to_stopline() +
                  input.intersection_info().intersection_length() +
                  kRoiDistAfterIntersection
            : 0.0;
    const double second_goal_s_dist =
        std::min(std::max(std::max(ego_speed * kSecondRoiTime, kSecondRoiDist),
                          dist_exit_intersection),
                 coarse_path_length - kRoiDistAfterIntersection);
    first_roi_index = target_lane.GetIndexFromS(ego_s + first_goal_s_dist).id;
    second_roi_index = target_lane.GetIndexFromS(ego_s + second_goal_s_dist).id;
    const int point_size = target_lane.num_points();
    if (first_roi_index < 0 || first_roi_index >= point_size ||
        second_roi_index < 0 || second_roi_index >= point_size || ego_index < 0 ||
        ego_index >= point_size) {
      return false;
    }
    return true;
  }

  bool GoalSampleGenerator::IsConsiderIntersection(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &input) const {
    const double kDistConsiderEnterIntersection = 50.0;
    const bool is_consider_intersection =
        input.intersection_info().dist_to_stopline() <
            kDistConsiderEnterIntersection ||
        input.intersection_info().is_in_intersection();
    return is_consider_intersection;
  }

  bool GoalSampleGenerator::GenerateGoalSampleForReverseLane(
      const npp::framework::Frame *frame,
      const double confidence_ego_in_reverse_lane,
      const double coarse_path_length,
      std::vector<reference_path_generator::GoalInfo> &goal_info) const {
    const auto virtual_lane_mgr =
        frame->session()->ego_prediction_context().virtual_lane_manager();
    const bool is_current_lane_refpath =
        virtual_lane_mgr->current_lane()->refline().lane_path_type() ==
            npp::common::IS_REFPATH;
    const double kConfidenceThresh = 0.1;
    MDEBUG_JSON_ADD_ITEM(confidence_ego_in_reverse_lane,
                         confidence_ego_in_reverse_lane, 0)
    MDEBUG_JSON_ADD_ITEM(is_current_lane_refpath, is_current_lane_refpath, 0)
    if ((confidence_ego_in_reverse_lane > kConfidenceThresh) &&
        is_current_lane_refpath) {
      goal_info.clear();
      const double kAwayReverseLaneOffset = -0.8;
      const double kSecondRoiTime = 4.0;
      const double kSecondRoiDist = 40.0;
      const double ego_speed =
          frame->ego_prediction_context().ego_state()->velocity();
      goal_info.push_back(
          {std::min(std::max(ego_speed * kSecondRoiTime, kSecondRoiDist),
                    coarse_path_length),
           kAwayReverseLaneOffset * confidence_ego_in_reverse_lane, true});
      goal_info.push_back({coarse_path_length, 0.0, true});
      return true;
    }
    return false;
  }
} // namespace reference_path_generator