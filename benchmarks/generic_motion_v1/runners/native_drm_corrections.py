"""Exact CPU-only correctness repairs to the preserved upstream DRM source.

An endpoint connection failure is an edge failure, not a colliding roadmap
vertex. Negative IDs are endpoint-edge tags and never enter the native graph.
No CUDA kernel, roadmap, collision radius or search budget is changed.
"""

CORRECTIONS = (
    ("  attempts_to_leave_node.clear();\n",
     """  attempts_to_leave_node.clear();
  // Heuristic query-local vertex exclusions must not leak into later queries.
  collision_set_ = geometry_invalid_nodes_;
  collision_set_.insert(environment_collision_nodes_.begin(),
                        environment_collision_nodes_.end());
  // Endpoint tags belong to this query, not the persistent scene collision map.
  for (auto it = forbidden_edge_set_.begin(); it != forbidden_edge_set_.end();) {
    if ((*it)[0] < 0 || (*it)[1] < 0) it = forbidden_edge_set_.erase(it);
    else ++it;
  }
"""),
    ("  // Get start node in roadmap.\n",
     """  // Endpoint-local rejection must not remove a valid intermediate vertex.
  auto endpoint_candidate = [&](const Eigen::VectorXf& q,
                                Eigen::VectorXf* closest, int endpoint_id) {
    int32_t best = -1;
    double distance = std::numeric_limits<double>::infinity();
    for (const auto& entry : drm_.id_to_node_map) {
      if (collision_set_.count(entry.first) ||
          forbidden_edge_set_.count({endpoint_id, entry.first})) continue;
      const double value = Distance(entry.second, q);
      if (value < distance) { distance = value; best = entry.first; }
    }
    if (best >= 0) *closest = drm_.id_to_node_map.at(best);
    return best;
  };

  // Get start node in roadmap.
"""),
    ("    start_id = GetClosestNonCollidingConfigurationInRoadMap(\n        start_configuration, &closest_start_joint);",
     "    start_id = endpoint_candidate(start_configuration, &closest_start_joint, -1);"),
    ("    collision_set_.insert(start_id);",
     "    forbidden_edge_set_.insert({-1, start_id});"),
    ("    target_id = GetClosestNonCollidingConfigurationInRoadMap(\n        target_configuration, &closest_target_joint);",
     "    target_id = endpoint_candidate(target_configuration, &closest_target_joint, -2);"),
    ("    collision_set_.insert(target_id);",
     "    forbidden_edge_set_.insert({-2, target_id});"),
    ("""      auto it = std::find(node_id_path.begin(), node_id_path.end(),
                          reduced_node_id_path[ii - 1]);
      int32_t node1 = *it;
      int32_t node2 = *(it + 1);
      if (node2 == -1) {
        // this indicates the target node.
        node2 = -2;
      }
""",
     """      int32_t node1 = reduced_node_id_path[ii - 1];
      int32_t node2 = reduced_node_id_path[ii];
      if (ii == reduced_node_id_path.size() - 1 && node2 == -1) node2 = -2;
"""),
    ("""      if (attempts_to_leave_node.find(node1) == attempts_to_leave_node.end()) {
        attempts_to_leave_node[node1] = 1;
      } else {
        attempts_to_leave_node.at(node1) += 1;
      }

      // insert node into collision set if too many outgoing edges are blocked.

      if (attempts_to_leave_node.at(node1) >=
          options_.max_num_blocked_edges_before_discard_node) {
        collision_set_.insert(node1);
      }
""",
     """      // Endpoint-edge failure does not invalidate its roadmap vertex.
      if (node1 >= 0 && node2 >= 0) {
        ++attempts_to_leave_node[node1];
        if (attempts_to_leave_node.at(node1) >=
            options_.max_num_blocked_edges_before_discard_node) {
          collision_set_.insert(node1);
        }
      }
"""),
    ("  // Clear reachability flag for secondary target joints.\n",
     """  if (!drm_.id_to_node_map.count(start_node_id) ||
      !drm_.id_to_node_map.count(target_node_id) ||
      collision_set_.count(start_node_id) || collision_set_.count(target_node_id))
    return false;
  if (start_node_id == target_node_id) return true;

  // Clear reachability flag for secondary target joints.
"""),
    ("""  int num_configurations_to_check =
      distance_between_configurations / options_.online_edge_step_size + 1;
  Eigen::MatrixXf configurations(num_joints_, num_configurations_to_check);
  const Eigen::VectorXf edge_direction =
      (next_configuration - current_configuration) /
      distance_between_configurations;
  for (int ii = 0; ii < num_configurations_to_check; ++ii) {
    const Eigen::VectorXf intermediate_configuration =
        current_configuration +
        ii * options_.online_edge_step_size * edge_direction;
    configurations.col(ii) = intermediate_configuration;
  }
""",
     """  if (!current_configuration.allFinite() || !next_configuration.allFinite() ||
      !std::isfinite(distance_between_configurations) ||
      !std::isfinite(options_.online_edge_step_size) ||
      options_.online_edge_step_size <= 0) return false;
  const int intervals = std::max(1, static_cast<int>(
      std::ceil(distance_between_configurations / options_.online_edge_step_size)));
  Eigen::MatrixXf configurations(num_joints_, intervals + 1);
  for (int ii = 0; ii <= intervals; ++ii) {
    const float fraction = static_cast<float>(ii) / intervals;
    configurations.col(ii) = (1.0f - fraction) * current_configuration +
                             fraction * next_configuration;
  }
"""),
)


def correct_source(source):
    for old, new in CORRECTIONS:
        if source.count(old) != 1:
            raise ValueError('Upstream DRM no longer matches exact reviewed correctness repair')
        source = source.replace(old, new, 1)
    return source
