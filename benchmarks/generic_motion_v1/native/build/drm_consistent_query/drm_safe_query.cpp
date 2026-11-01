// Local binding for an isolated build of the upstream DRM CPU translation unit.
// CUDA kernels and the installed csdecomp package are never replaced.
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "planning/drm_planner.h"

namespace py = pybind11;
using csdecomp::DrmPlanner;

PYBIND11_MODULE(_drm_safe_query, m) {
  m.doc() = "Original DRM CPU query with non-aborting exhausted-search returns";
  m.def("layout", [](const DrmPlanner& p) {
    py::dict d;
    d["num_joints"] = p.num_joints_;
    d["nodes"] = p.drm_.id_to_node_map.size();
    d["collision_nodes"] = p.collision_set_.size();
    d["loaded"] = p.drm_loaded_;
    d["initialized"] = p.is_initialized_;
    d["max_number_planning_attempts"] = p.options_.max_number_planning_attempts;
    d["max_iterations_steering_to_node"] = p.options_.max_iterations_steering_to_node;
    d["max_nodes_to_expand"] = p.options_.max_nodes_to_expand;
    d["online_edge_step_size"] = p.options_.online_edge_step_size;
    return d;
  });
  m.def("closest_node", [](DrmPlanner& p, const Eigen::VectorXf& desired) {
    Eigen::VectorXf closest(desired.size());
    const auto id = p.GetClosestNonCollidingConfigurationInRoadMap(desired, &closest);
    return py::make_tuple(id, id < 0 ? py::none() : py::cast(closest));
  });
  m.def("plan", [](DrmPlanner& p, const Eigen::VectorXf& start,
                   const Eigen::VectorXf& goal, const csdecomp::Voxels& voxels,
                   float radius) {
    std::vector<Eigen::VectorXf> path;
    bool success = p.Plan(start, goal, voxels, radius, &path);
    return py::make_tuple(success, path);
  });
  m.def("path_exists", [](DrmPlanner& p, int start, int goal) {
    return p.CheckPathExists(start, goal, {}, nullptr);
  });
  m.def("steer", [](DrmPlanner& p, const Eigen::VectorXf& start,
                    const Eigen::VectorXf& goal, const csdecomp::Voxels& voxels,
                    float radius) {
    return p.SteerFunctionIsSafe(start, goal, voxels, radius);
  });
}
