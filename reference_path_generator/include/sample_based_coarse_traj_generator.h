#pragma once
#include <cmath>
#include <memory>
#include <vector>

#include "core/modules/common/ego_prediction_debug_logger.h"
#include "core/modules/common/math/box2d.h"
#include "core/modules/common/math/obb.h"
#include "core/modules/path_planner/path_planner_preprocessor/frenet_forward_simulator.h"
#include "core/modules/path_planner/path_planner_preprocessor/path_planner_constants.hpp"
#include "core/modules/path_planner/path_planner_preprocessor/pnc_path_with_ppp_input.h"
#include "core/proto/cartesian_road_boundaries.pb.h"
#include "core/proto/planning_proto_data.pb.h"
#include "goal_sample_generator.h"
#include "is_collision.h"

namespace reference_path_generator {
bool SampleBasedCoarseTrajSearch(
    const std::vector<GoalInfo> &goal_info,
    const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
    const std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs,
    const std::vector<npp::planning_math::Obbox> &obbs,
    npp::pnc::SampleCoarseData *output);

class SampleBasedCoarseTrajGenerator {
 public:
  SampleBasedCoarseTrajGenerator() = default;
  ~SampleBasedCoarseTrajGenerator() = default;

  npp::pnc::ForwardSimAgent GenSimAgent(
      const double dt, const int sim_size,
      const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input) {
    constexpr double kPreviewTime = 2.5;
    npp::pnc::State state_with_s_offset;
    state_with_s_offset.CopyFrom(ppp_input.init_state());
    npp::pnc::ForwardSimAgent ego_agent(state_with_s_offset, kPreviewTime,
                                        ppp_input.agent_params().wheel_base(),
                                        dt, sim_size);
    const double steer_angle_to_front_angle_radians =
        ppp_input.agent_params().steer_angle_to_front_angle_radians();
    ego_agent.SetDeltaAndGammaLimit(
        path_planner::kDelatLimitSteerDeg * steer_angle_to_front_angle_radians,
        path_planner::kGammaLimitSteerDeg * steer_angle_to_front_angle_radians);
    return ego_agent;
  }

  void GenerateSinCurv(const double x0, const double y0, const double x1,
                       const double y1, std::vector<double> *param) {
    // y = a + b * sin(c * x + d)
    param->resize(4);
    // a
    param->at(0) = (y0 + y1) / 2.0;
    // b
    param->at(1) = (y1 - y0) / 2.0;
    // c
    param->at(2) = M_PI / (x1 - x0);
    // d
    param->at(3) = -M_PI_2 * (x0 + x1) / (x1 - x0);
  }

  bool SearchSamplePath(
      const std::vector<GoalInfo> &goal_info,
      const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
      const std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs,
      const std::vector<npp::planning_math::Obbox> &obbs,
      npp::pnc::SampleCoarseData *output) {
    double start_s = input.init_state().frenet_state().s();
    double end_s = start_s;
    double start_l = input.init_state().frenet_state().l();
    double end_l = start_l;
    std::vector<double> sample_target_l;
    const double sample_ratio = 0.5;
    const double ratio = (goal_info.size() > 1U) ? sample_ratio : 1.0;
    for (const auto &goal : goal_info) {
      start_s = end_s;
      end_s = input.init_state().frenet_state().s() + goal.relative_to_ego_s;
      start_l = end_l;
      MDEBUG_JSON_ADD_ITEM(goal1_real_l, end_l, 0)
      end_l = goal.relative_to_ref_l;
      sample_target_l.clear();
      GetSampleTargetL(input, end_l, sample_target_l, ratio);
      if (!GenerateSinCurvWithGoalPoint(sample_target_l, input, fcs, obbs,
                                        start_s, start_l, end_s, end_l,
                                        output)) {
        return false;
      }
    }
    return true;
  }

  bool GenerateSinCurvWithGoalPoint(
      const std::vector<double> &sample_target_l,
      const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
      const std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs,
      const std::vector<npp::planning_math::Obbox> &obbs, const double start_s,
      const double start_l, const double end_s, double &used_end_l,
      npp::pnc::SampleCoarseData *output) {
    for (const auto &target_l : sample_target_l) {
      auto *coarse_path = output->add_coarse_sample_path()->mutable_path();
      const double end_l = target_l;
      std::vector<double> sin_param;
      GenerateSinCurv(start_s, start_l, end_s, end_l, &sin_param);
      const double sample_size = 40;
      const double s_gap = (end_s - start_s) / sample_size;
      bool generate_sin_success = true;
      for (int i = 0; i <= sample_size; ++i) {
        const double s = start_s + s_gap * i;
        const double l =
            sin_param[0] + sin_param[1] * sin(sin_param[2] * s + sin_param[3]);
        npp::planning_math::Vec2d pt;
        if (!fcs->ToCartPoint({s, l}, &pt)) {
          generate_sin_success = false;
          break;
        }
        auto *state = coarse_path->Add();
        state->set_x(pt.x());
        state->set_y(pt.y());
      }
      // calculate theta
      for (int i = 0; i < coarse_path->size(); ++i) {
        if (i + 1 == coarse_path->size()) {
          const auto &cur_point = (*coarse_path)[i];
          const auto &pre_point = (*coarse_path)[i - 1];
          const double theta = std::atan2(cur_point.y() - pre_point.y(),
                                          cur_point.x() - pre_point.x());
          (*coarse_path)[i].set_theta(theta);
        } else {
          const auto &cur_point = (*coarse_path)[i];
          const auto &next_point = (*coarse_path)[i + 1];
          const double theta = std::atan2(next_point.y() - cur_point.y(),
                                          next_point.x() - cur_point.x());
          (*coarse_path)[i].set_theta(theta);
        }
      }
      if (generate_sin_success &&
          npp::IsNotCollision(input.agent_params(), *coarse_path, obbs)) {
        auto *coarse_path_final = output->mutable_coarse_path_final();
        coarse_path_final->MergeFrom(*coarse_path);
        used_end_l = target_l;
        MDEBUG_JSON_ADD_ITEM(last_generated_goal_target_l, target_l, 0);
        MDEBUG_JSON_ADD_ITEM(coarse_path_final_size,
                             coarse_path_final->size(), 0);
        return true;
      }
    }
    return false;
  }

  void GetSampleTargetL(
      const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
      const double target_l, std::vector<double> &sample_target_l,
      const double ratio) {
    const int sample_size = 7;
    sample_target_l.reserve(sample_size);
    const double SAMPLE_GAP = 0.6 * ratio;
    const double SAMPLE_GAP_2 = 1.2 * ratio;
    const double SAMPLE_GAP_3 = 1.8 * ratio;
    if (input.lane_decider_info().lc_status() == npp::pnc::LeftLaneChange) {
      for (int i = 0; i < sample_size; ++i) {
        sample_target_l.push_back(target_l - SAMPLE_GAP * i);
      }
    } else if (input.lane_decider_info().lc_status() ==
               npp::pnc::RightLaneChange) {
      for (int i = 0; i < sample_size; ++i) {
        sample_target_l.push_back(target_l + SAMPLE_GAP * i);
      }
    } else {
      sample_target_l.push_back(target_l);
      sample_target_l.push_back(target_l - SAMPLE_GAP);
      sample_target_l.push_back(target_l + SAMPLE_GAP);
      sample_target_l.push_back(target_l - SAMPLE_GAP_2);
      sample_target_l.push_back(target_l + SAMPLE_GAP_2);
      sample_target_l.push_back(target_l - SAMPLE_GAP_3);
      sample_target_l.push_back(target_l + SAMPLE_GAP_3);
    }
  }
};
} // namespace reference_path_generator