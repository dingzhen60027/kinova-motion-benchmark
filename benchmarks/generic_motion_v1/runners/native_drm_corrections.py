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


# The quantized offline cache is a broad phase, not an exact classification for
# differently centered online spheres. Keep graph/model/CUDA code unchanged and
# account for all exact node checks inside the native online Plan call.
QUERY_CONSISTENCY_CORRECTIONS = (
    ("  int success_flag{0};\n",
     """  // Reconcile the broad-phase cache with this query's actual sphere centers
  // and radius. This also detects colliding nodes missed by the quantized map.
  // Cache edge hits are likewise only broad phase; native lazy checking below
  // validates every edge before any path is returned.
  environment_collision_nodes_.clear();
  collision_set_ = geometry_invalid_nodes_;
  forbidden_edge_set_.clear();
  std::vector<int32_t> node_ids;
  Eigen::MatrixXf node_configurations(num_joints_, drm_.id_to_node_map.size());
  int node_column = 0;
  for (const auto& entry : drm_.id_to_node_map) {
    node_ids.push_back(entry.first);
    node_configurations.col(node_column++) = entry.second;
  }
  if (!node_ids.empty()) {
    const auto free_nodes = checkCollisionFreeVoxelsCuda(
        &node_configurations, &root_p_voxels, online_voxel_radius, &mplant_,
        inspector_.robot_geometry_ids);
    for (size_t index = 0; index < node_ids.size(); ++index) {
      if (!free_nodes.at(index)) {
        environment_collision_nodes_.insert(node_ids[index]);
        collision_set_.insert(node_ids[index]);
      }
    }
  }

  int success_flag{0};
"""),
    ("      if (IsEdgeBlocked(next_node, node)) {\n",
     "      if (IsEdgeBlocked(node, next_node)) {\n"),
    ("  // Find path in roadmap from start node to target node.\n",
     """  // The first two visible vertices can belong to different components.
  // That does not establish failure of every endpoint attachment. Examine
  // further candidates under the same per-endpoint steering-check limit.
  bool endpoints_connected = CheckPathExists(start_id, target_id, {}, nullptr);
  if (!endpoints_connected) {
    auto ordered_candidates = [&](const Eigen::VectorXf& endpoint, int tag,
                                  int already_visible) {
      std::vector<std::pair<double, int32_t>> candidates;
      for (const auto& entry : drm_.id_to_node_map) {
        if (entry.first == already_visible || collision_set_.count(entry.first) ||
            forbidden_edge_set_.count({tag, entry.first})) continue;
        candidates.emplace_back(Distance(endpoint, entry.second), entry.first);
      }
      std::sort(candidates.begin(), candidates.end());
      return candidates;
    };
    auto starts = ordered_candidates(start_configuration, -1, start_id);
    auto goals = ordered_candidates(target_configuration, -2, target_id);
    std::vector<int32_t> visible_starts{start_id}, visible_goals{target_id};
    size_t start_cursor = 0, goal_cursor = 0;
    size_t start_checks = steer_to_start_iterations + 1;
    size_t goal_checks = steer_to_target_iterations + 1;
    const size_t limit = options_.max_iterations_steering_to_node;
    while (!endpoints_connected &&
           ((start_checks < limit && start_cursor < starts.size()) ||
            (goal_checks < limit && goal_cursor < goals.size()))) {
      if (start_checks < limit && start_cursor < starts.size()) {
        const int32_t candidate = starts[start_cursor++].second;
        ++start_checks;
        if (SteerFunctionIsSafe(start_configuration, drm_.id_to_node_map.at(candidate),
                                root_p_voxels, online_voxel_radius)) {
          visible_starts.push_back(candidate);
          for (const auto goal : visible_goals) {
            if (CheckPathExists(candidate, goal, {}, nullptr)) {
              start_id = candidate; target_id = goal; endpoints_connected = true;
              break;
            }
          }
        } else {
          forbidden_edge_set_.insert({-1, candidate});
        }
      }
      if (!endpoints_connected && goal_checks < limit && goal_cursor < goals.size()) {
        const int32_t candidate = goals[goal_cursor++].second;
        ++goal_checks;
        if (SteerFunctionIsSafe(target_configuration, drm_.id_to_node_map.at(candidate),
                                root_p_voxels, online_voxel_radius)) {
          visible_goals.push_back(candidate);
          for (const auto start : visible_starts) {
            if (CheckPathExists(start, candidate, {}, nullptr)) {
              start_id = start; target_id = candidate; endpoints_connected = true;
              break;
            }
          }
        } else {
          forbidden_edge_set_.insert({-2, candidate});
        }
      }
    }
  }

  // Find path in roadmap from start node to target node.
"""),
    ("  if (CheckPathExists(start_id, target_id, {}, nullptr)) {\n",
     "  if (endpoints_connected) {\n"),
    ('          "[DRMPlanner] No collision-free path exists in roadmap.");\n',
     '          "[DRMPlanner] No route found through tested roadmap connections.");\n'),
)


def reconcile_query_source(source):
    """Apply only after correct_source; fail closed on unreviewed source drift."""
    for old, new in QUERY_CONSISTENCY_CORRECTIONS:
        if source.count(old) != 1:
            raise ValueError('Upstream DRM no longer matches exact reviewed query consistency repair')
        source = source.replace(old, new, 1)
    return source
