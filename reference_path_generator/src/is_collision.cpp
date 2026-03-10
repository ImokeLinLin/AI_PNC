#include "is_collision.h"
namespace npp {

bool IsCollision(const npp::pnc::RoadBoundary &rbs,
                 const npp::planning_math::Box2d &ego_box,
                 pnc::common::LineSegment2d *segment) {
  npp::planning_math::LineSegment2d line({0.0, 0.0}, {0.0, 0.0});
  for (int i = 0; i < rbs.segment_size(); ++i) {
    const auto &rb_segment = rbs.segment(i);
    for (int j = 0; j + 1 < rb_segment.pt_size(); ++j) {
      const auto &pt1 = rb_segment.pt(j);
      const auto &pt2 = rb_segment.pt(j + 1);
      line.Set(pt1.x(), pt1.y(), pt2.x(), pt2.y());
      if (ego_box.HasOverlap(line)) {
        if (segment != nullptr) {
          auto *start_pt = segment->mutable_start();
          auto *end_pt = segment->mutable_end();
          start_pt->set_x(line.start().x());
          start_pt->set_y(line.start().y());
          end_pt->set_x(line.end().x());
          end_pt->set_y(line.end().y());
        }
        return true;
      }
    }
  }
  return false;
}

bool IsNotCollision(const npp::pnc::AgentParams &agent_params,
                    const path_planner::ProtoStates &coarse_path,
                    const std::vector<npp::pnc::RoadBoundary> &boundarys,
                    pnc::common::LineSegment2d *segment) {
  const int traj_size = coarse_path.size();
  for (int pt_index = 0; pt_index < traj_size; pt_index++) {
    const auto &cur_point = coarse_path[pt_index];
    npp::planning_math::Box2d ego_box({cur_point.x(), cur_point.y()},
                                      cur_point.theta(), agent_params.length(),
                                      agent_params.width());
    for (const auto &boundary : boundarys) {
      if (IsCollision(boundary, ego_box, segment)) {
        return false;
      }
    }
  }
  return true;
}

bool IsNotCollision(const npp::pnc::AgentParams &agent_params,
                    const path_planner::ProtoStates &coarse_path,
                    const std::vector<planning_math::Obbox> &obbs,
                    pnc::common::LineSegment2d *segment) {
  const int traj_size = coarse_path.size();
  for (int pt_index = 0; pt_index < traj_size; pt_index++) {
    const auto &cur_point = coarse_path[pt_index];
    npp::planning_math::Box2d ego_box({cur_point.x(), cur_point.y()},
                                      cur_point.theta(), agent_params.length(),
                                      agent_params.width());
    for (const auto &obb : obbs) {
      if (obb.IsCollision(ego_box, segment)) {
        return false;
      }
    }
  }
  return true;
}

} // namespace npp