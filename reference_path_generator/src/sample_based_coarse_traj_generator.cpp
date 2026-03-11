#include "sample_based_coarse_traj_generator.h"
namespace reference_path_generator {
bool SampleBasedCoarseTrajSearch(
    const std::vector<GoalInfo> &goal_info,
    const npp::pnc::path_plan_pre::PPPreprocessorInput &input,
    const std::shared_ptr<path_planner::PncPathWithPPPInput> &fcs,
    const std::vector<npp::planning_math::Obbox> &obbs,
    npp::pnc::SampleCoarseData *output) {
  reference_path_generator::SampleBasedCoarseTrajGenerator
      sample_based_coarse_traj_generator;
  return sample_based_coarse_traj_generator.SearchSamplePath(goal_info, input,
                                                            fcs, obbs, output);
}
} // namespace reference_path_generator