// Benchmark Task bridge to the unmodified, vendored ros-industrial STOMP core.
// Copyright 2026. This bridge is distributed under the repository license.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <stomp/stomp.h>
#include <stomp/utils.h>

namespace py = pybind11;
class PythonTask final : public stomp::Task {
 public:
  explicit PythonTask(py::object task) : task_(std::move(task)) {}
  bool generateNoisyParameters(const Eigen::MatrixXd& q, std::size_t,
      std::size_t, int iteration, int rollout, Eigen::MatrixXd& noisy,
      Eigen::MatrixXd& noise) override {
    py::tuple result = task_.attr("noise")(q.transpose(), iteration, rollout);
    noisy = result[0].cast<Eigen::MatrixXd>().transpose();
    noise = result[1].cast<Eigen::MatrixXd>().transpose();
    return true;
  }
  bool computeNoisyCosts(const Eigen::MatrixXd& q, std::size_t,
      std::size_t, int iteration, int rollout, Eigen::VectorXd& costs,
      bool& valid) override {
    py::tuple result = task_.attr("cost")(q.transpose(), iteration, rollout);
    costs = result[0].cast<Eigen::VectorXd>();
    valid = result[1].cast<bool>();
    return true;
  }
  bool computeCosts(const Eigen::MatrixXd& q, std::size_t first,
      std::size_t count, int iteration, Eigen::VectorXd& costs,
      bool& valid) override {
    return computeNoisyCosts(q, first, count, iteration, -1, costs, valid);
  }
  bool filterParameterUpdates(std::size_t, std::size_t, int iteration,
      const Eigen::MatrixXd& q, Eigen::MatrixXd& delta) override {
    delta = task_.attr("update")(q.transpose(), delta.transpose(), iteration)
                .cast<Eigen::MatrixXd>().transpose();
    return true;
  }
  void postIteration(std::size_t, std::size_t, int iteration, double cost,
      const Eigen::MatrixXd& q) override {
    task_.attr("post")(iteration, cost, q.transpose());
  }
  void done(bool valid, int iterations, double cost,
      const Eigen::MatrixXd& q) override {
    task_.attr("done")(valid, iterations, cost, q.transpose());
  }
 private:
  py::object task_;
};

PYBIND11_MODULE(_generic_stomp, module) {
  module.def("solve", [](const Eigen::MatrixXd& initial, py::object task,
      int iterations, int after_valid, int rollouts, double sensitivity,
      double control_weight) {
    if (initial.cols() != 7 || initial.rows() < 8 || !initial.allFinite())
      throw std::invalid_argument("STOMP expects finite N x 7 with N >= 8");
    if (iterations < 1 || after_valid < 0 || rollouts < 2 ||
        sensitivity <= 0 || control_weight < 0)
      throw std::invalid_argument("Invalid STOMP configuration");
    stomp::StompConfiguration config;
    config.num_iterations = iterations;
    config.num_iterations_after_valid = after_valid;
    config.num_timesteps = initial.rows();
    config.num_dimensions = initial.cols();
    config.delta_t = 1.0;
    config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;
    config.exponentiated_cost_sensitivity = sensitivity;
    config.num_rollouts = rollouts;
    config.max_rollouts = rollouts + 1;  // no history-dependent rollout reuse
    config.control_cost_weight = control_weight;
    auto adapter = std::make_shared<PythonTask>(task);
    stomp::Stomp optimizer(config, adapter);
    Eigen::MatrixXd result;
    const bool valid = optimizer.solve(Eigen::MatrixXd(initial.transpose()), result);
    return py::make_tuple(valid, Eigen::MatrixXd(result.transpose()));
  });
  module.def("smoothing_matrix", [](int n) {
    if (n < 8) throw std::invalid_argument("At least eight points required");
    Eigen::MatrixXd matrix;
    stomp::generateSmoothingMatrix(n, 1.0, matrix);
    return matrix;
  });
}
