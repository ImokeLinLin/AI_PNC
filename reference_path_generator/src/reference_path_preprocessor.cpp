#include "reference_path_preprocessor.h"

#include "core/modules/common/math/pnc_kdpath.h"
#include "core/modules/common/math/vec2d.h"
#include "core/modules/path_planner/path_planner_ilqr/include/path_planner_model.h"
#include "core/modules/path_planner/path_planner_preprocessor/path_planner_constants.hpp"
#include "pnc_path_with_ppp_input.h"
#include "thirdparty/fplus/fplus.hpp"
namespace reference_path_generator {
void ReferencePathPreprocessor::InitFcs(
    const path_planner::ProtoRefPoints &ref_points,
    const npp::pnc::RefLinesInfo &ref_lines_info,
    std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs) {
  fcs = std::make_unique<path_planner::PncPathWithPPPInput>();
  fcs->Init(ref_points);
  fcs->SetFrenetLimit(ref_lines_info.s_begin(), ref_lines_info.s_end());
  fcs_ = fcs;
}

double ReferencePathPreprocessor::CalCoarsePathLength(
    const path_planner::ProtoRefPoints &ref_points,
    const double lat_path_length, const npp::pnc::State &init_state,
    const bool is_in_uturn) {
  const double ref_path_relative_length =
      fcs_->original_accumulated_s().back() - init_state.frenet_state().s();
  constexpr double kGeneralCoarseLength = 40.0;
  constexpr double kUturnCoarseLength = 15.0;
  const double kMinCoarseLength =
      is_in_uturn ? kUturnCoarseLength : kGeneralCoarseLength;
  //2025/08/20 lbfgs规划长度使用1.2倍粗轨迹长度，避免远端参考线不佳导致影响整体轨迹质量
  const double lat_path_length_gain = 1.2;
  double expect_length =
      std::max(kMinCoarseLength, lat_path_length * lat_path_length_gain);
  if (expect_length >= ref_path_relative_length) {
    const double kFrenetProtection = 0.5;
    return ref_path_relative_length - kFrenetProtection;
  }
  int match_index = -1;
  for (int i = 0; i < ref_points.size(); ++i) {
    if (ref_points[i].s() > expect_length + init_state.frenet_state().s()) {
      match_index = i;
      break;
    }
  }

  if (match_index == -1) {
    return expect_length;
  }

  if (ref_points[match_index].is_in_intersection()) {
    int end_index = -1;
    for (int i = match_index; i < ref_points.size(); ++i) {
      if (!ref_points[i].is_in_intersection()) {
        end_index = i;
        break;
      }
    }
    if (end_index != -1) {
      expect_length = ref_points[end_index].s() - init_state.frenet_state().s();
    }
  }
  if (expect_length < 0.0) {
    expect_length = 0.0;
  }
  return expect_length;
}

void ReferencePathPreprocessor::Update(
    npp::pnc::path_plan_pre::PPPreprocessorInput &input,
    npp::pnc::path_plan_pre::PPPreprocessorOutput &output, const bool is_hpa) {
  npp::pnc::TrajInfo *traj_info = output.mutable_traj_info();

  // second step: fill init_state and add vel protection
  SaveInitVelRatio(input, output);
  UpdateInputInitState(output.traj_info().vel_ratio(), input);

  // third step: cal traj info eg ddp path length, ref line length
  UpdateDdpPathInfo(input, traj_info);
  UpdateRefLinesInfo(traj_info);

  const double final_length = GenFinalLength(input.init_state(), traj_info);
  traj_info->set_final_length(final_length);

  const double coarse_path_length =
      CalCoarsePathLength(input.ref_lines_ori(), final_length,
                          input.init_state(), (input.is_enhance_lb_in_uturn() || is_hpa));

  GenerateReferenceBBox(input.init_state().frenet_state().s(),
                        input.init_state().frenet_state().l(),
                        coarse_path_length, &ref_box_);
  traj_info->set_coarse_path_length(coarse_path_length);
}

double ReferencePathPreprocessor::GenFinalLength(
    const npp::pnc::State &init_state, const npp::pnc::TrajInfo *traj_info) {
  const double init_s = init_state.frenet_state().s();
  const double ddp_length_with_protection =
      fmax(path_planner::kLatPlanMinLen, traj_info->ddp_traj_length());
  const double reference_line_length = traj_info->ref_path_length() - init_s -
                                       path_planner::kFrenetLengthProtect;
  const double final_length =
      fmin(reference_line_length, ddp_length_with_protection);
  return final_length;
}

void ReferencePathPreprocessor::UpdateRefLinesInfo(
    npp::pnc::TrajInfo *traj_info) {
  traj_info->set_ref_path_length(fcs_->original_accumulated_s().back());
}

void ReferencePathPreprocessor::UpdateDdpPathInfo(
    const npp::pnc::path_plan_pre::PPPreprocessorInput &ppp_input,
    npp::pnc::TrajInfo *traj_info) {
  const int &ddp_size = ppp_input.ddp_path_ori_size();
  double ddp_length = 0.0;
  if (ddp_size != 0) {
    ddp_length = ppp_input.ddp_path_ori(ddp_size - 1).frenet_state().s() -
                 ppp_input.ddp_path_ori(0).frenet_state().s();
  }
  traj_info->set_ddp_traj_length(ddp_length);
  traj_info->set_is_ddp_traj_short(ddp_length < path_planner::kLatPlanMinLen);
}

void ReferencePathPreprocessor::SaveInitVelRatio(
    const npp::pnc::path_plan_pre::PPPreprocessorInput &pppreprocessor_input,
    npp::pnc::path_plan_pre::PPPreprocessorOutput &output) {
  const double kVelRatioMax = 5.0;
  const double vel_with_protection = std::max(
      path_planner::kLatVelRatioMinVel, pppreprocessor_input.init_state().v());
  const double vel_ratio = fplus::clamp(
      1.0, kVelRatioMax, path_planner::kLatPlanMinVel / vel_with_protection);
  output.mutable_traj_info()->set_vel_ratio(vel_ratio);
}

void ReferencePathPreprocessor::UpdateInputInitState(
    const double vel_ratio,
    npp::pnc::path_plan_pre::PPPreprocessorInput &pppreprocessor_input) {
  // x, y, theta, v, a, t, curv, frent
  // will be copy from input state
  // delta, gamma is still empty
  auto *input_init_state = pppreprocessor_input.mutable_init_state();
  const double original_vel = input_init_state->v();
  const bool is_replan = pppreprocessor_input.scenario_info().is_replan();
  input_init_state->set_v(std::max(original_vel, path_planner::kLatPlanMinVel));
  if (is_replan || (!pppreprocessor_input.has_prev_ilqr_nearest_state())) {
    input_init_state->set_delta(
        std::atan(pppreprocessor_input.agent_params().wheel_base() *
                  pppreprocessor_input.init_state().curvature()));
    input_init_state->set_gamma(0.0);
  } else {
    const double delta_limit =
        lat_ilqr::ILqrLatModel::FunctionSafetyDeltaLimit(original_vel);
    const double omega_limit = lat_ilqr::ILqrLatModel::FunctionSafetyOmegaLimit(
        original_vel, vel_ratio);
    input_init_state->set_delta(
        fplus::clamp(-delta_limit, delta_limit,
                     pppreprocessor_input.prev_ilqr_nearest_state().delta()));
    input_init_state->set_gamma(
        fplus::clamp(-omega_limit, omega_limit,
                     pppreprocessor_input.prev_ilqr_nearest_state().gamma()));
  }
}

void ReferencePathPreprocessor::FilterAndTransToProto(
    const npp::pnc::RoadBoundary &rb, npp::pnc::RoadBoundary *res) const {
  res->Clear();
  for (const auto &segment : rb.segment()) {
    bool has_overlap = false;
    for (int i = 0; i < segment.pt_size(); i++) {
      const npp::planning_math::Vec2d pt(segment.pt(i).x(), segment.pt(i).y());
      if (OverlapWithRefPathBox(pt, ref_box_)) {
        has_overlap = true;
        break;
      }
    }
    if (has_overlap) {
      auto *seg = res->add_segment();
      seg->CopyFrom(segment);
      seg->set_lat_decision(npp::pnc::common::LatObstacleDecisionType::NONE);
    }
  }
}

bool ReferencePathPreprocessor::OverlapWithRefPathBox(
    const npp::planning_math::Vec2d &pt,
    const std::vector<npp::planning_math::Box2d> &ref_path_box) const {
  for (const auto &box : ref_path_box) {
    if (box.IsPointIn(pt)) {
      return true;
    }
  }
  return false;
}

void ReferencePathPreprocessor::GenerateReferenceBBox(
    const double init_s, const double init_l, const double coarse_path_length,
    std::vector<npp::planning_math::Box2d> *res) {
  // 加20个点的保护是为了防止当total_ref_length过长，导致bbox_num过多，
  // 后续Filter遍历计算量过大
  const double bbox_length = std::fmax(
      path_planner::kRefBboxMinLength,
      coarse_path_length / static_cast<double>(path_planner::kLatIlqrHorizon));

  // 考虑init_l是为了仿真车变道过程中，ref_lines切换
  const double bbox_width =
      path_planner::kRefBboxDefaultWidth + std::fabs(init_l + init_l);
  const int bbox_num = std::ceil(coarse_path_length / bbox_length);
  if (bbox_num < 0) {
    res->clear();
    return;
  }
  res->resize(bbox_num);

  npp::pnc::RefPoint start_pt;
  npp::pnc::RefPoint end_pt;
  for (int i = 0; i < bbox_num; ++i) {
    const double start_s = init_s + i * bbox_length;
    const double end_s = start_s + bbox_length;

    fcs_->SampleReflinePoint(start_s, &start_pt);

    fcs_->SampleReflinePoint(end_s, &end_pt);
    npp::planning_math::LineSegment2d bbox_center_line(
        {start_pt.x(), start_pt.y()}, {end_pt.x(), end_pt.y()});
    npp::planning_math::Box2d bbox(bbox_center_line, bbox_width);
    bbox.LongitudinalExtend(path_planner::kRefBboxExtendLength);
    (*res)[i] = bbox;
  }
}

} // namespace reference_path_generator