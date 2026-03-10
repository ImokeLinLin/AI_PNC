#pragma once
#include <cmath>
#include <memory>
#include <vector>

#include "core/modules/common/math/box2d.h"
#include "core/modules/common/math/obb.h"
#include "core/modules/path_planner/path_planner_preprocessor/pnc_path_with_ppp_input.h"
#include "core/proto/cartesian_road_boundaries.pb.h"

namespace npp {

bool IsCollision(const npp::pnc::RoadBoundary &rbs,
                 const npp::planning_math::Box2d &ego_box,
                 pnc::common::LineSegment2d *segment = nullptr);
bool IsNotCollision(const npp::pnc::AgentParams &agent_params,
                    const path_planner::ProtoStates &coarse_path,
                    const std::vector<npp::pnc::RoadBoundary> &boundarys,
                    pnc::common::LineSegment2d *segment = nullptr);
bool IsNotCollision(const npp::pnc::AgentParams &agent_params,
                    const path_planner::ProtoStates &coarse_path,
                    const std::vector<planning_math::Obbox> &obbs,
                    pnc::common::LineSegment2d *segment = nullptr);
} // namespace npp