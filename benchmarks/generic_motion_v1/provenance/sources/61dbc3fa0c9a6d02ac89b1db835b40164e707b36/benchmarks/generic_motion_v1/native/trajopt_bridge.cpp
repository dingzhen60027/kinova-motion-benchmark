// Thin C ABI around the unmodified author TrajOpt BasicTrustRegionSQP.
// Only problem costs, exact endpoint bounds and callbacks are local.
#include "sco/optimizers.hpp"
#include "sco/expr_ops.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

typedef int (*DistanceCallback)(const double*, int, int, double*, double*);
typedef int (*IterationCallback)(const double*, int, int);

namespace {
class VelocityCost : public sco::Cost {
  sco::VarVector vars_;
  sco::QuadExpr expression_;
public:
  explicit VelocityCost(const sco::VarVector& vars) : Cost("joint_velocity_squared"), vars_(vars) {
    for (size_t i = 7; i < vars.size(); ++i) {
      sco::AffExpr difference(vars[i]);
      sco::exprDec(difference, vars[i-7]);
      sco::exprInc(expression_, sco::exprSquare(difference));
    }
  }
  double value(const std::vector<double>& x) {return expression_.value(x);}
  sco::ConvexObjectivePtr convex(const std::vector<double>&, sco::Model* model) {
    sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
    out->addQuadExpr(expression_);
    return out;
  }
};

class ClearanceConstraint : public sco::IneqConstraint {
  sco::VarVector vars_;
  int count_;
  double margin_;
  DistanceCallback callback_;
public:
  ClearanceConstraint(const sco::VarVector& vars, int count, double margin,
                      DistanceCallback callback)
      : vars_(vars), count_(count), margin_(margin), callback_(callback) {
    setName("canonical_collision_clearance");
  }
  std::vector<double> value(const std::vector<double>& x) {
    std::vector<double> distances(count_);
    if (callback_(x.data(), (int)vars_.size(), 0, distances.data(), NULL))
      throw std::runtime_error("Python distance callback failed or reached deadline");
    for (double& value : distances) value = margin_ - value;
    return distances;
  }
  sco::ConvexConstraintsPtr convex(const std::vector<double>& x, sco::Model* model) {
    std::vector<double> distances(count_), gradient(count_ * vars_.size());
    if (callback_(x.data(), (int)vars_.size(), 1, distances.data(), gradient.data()))
      throw std::runtime_error("Python distance derivative callback failed or reached deadline");
    sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
    for (int r = 0; r < count_; ++r) {
      sco::AffExpr expr(margin_ - distances[r]);
      for (size_t c = 0; c < vars_.size(); ++c) {
        const double g = gradient[r * vars_.size() + c];
        expr.constant += g * x[c];
        if (g != 0) {expr.vars.push_back(vars_[c]); expr.coeffs.push_back(-g);}
      }
      out->addIneqCnt(expr);
    }
    return out;
  }
};
}

extern "C" int trajopt_solve(int count, const double* initial,
    const double* lower, const double* upper, int constraint_count,
    DistanceCallback distance_callback, IterationCallback iteration_callback,
    const double* settings, double* solution, double* statistics,
    char* error, int error_size) {
  try {
    if (count < 14 || count % 7 || constraint_count < 1)
      throw std::invalid_argument("Invalid seven-joint trajectory dimensions");
    sco::OptProbPtr problem(new sco::OptProb());
    std::vector<std::string> names;
    for (int i = 0; i < count; ++i) names.push_back("q_" + std::to_string(i));
    sco::VarVector vars = problem->createVariables(names,
        std::vector<double>(lower, lower+count), std::vector<double>(upper, upper+count));
    problem->addCost(sco::CostPtr(new VelocityCost(vars)));
    problem->addIneqConstraint(sco::ConstraintPtr(new ClearanceConstraint(
        vars, constraint_count, settings[0], distance_callback)));
    sco::BasicTrustRegionSQP optimizer(problem);
    optimizer.max_iter_ = settings[1];
    optimizer.trust_box_size_ = settings[2];
    optimizer.cnt_tolerance_ = settings[3];
    optimizer.merit_error_coeff_ = settings[4];
    optimizer.max_merit_coeff_increases_ = settings[5];
    optimizer.min_approx_improve_ = settings[6];
    optimizer.initialize(std::vector<double>(initial, initial+count));
    int iteration = 0;
    optimizer.addCallback([&](sco::OptProb*, sco::DblVec& x) {
      if (iteration_callback(x.data(), count, iteration++))
        throw std::runtime_error("Python iteration callback failed or reached deadline");
    });
    int status = (int)optimizer.optimize();
    sco::OptResults& result = optimizer.results();
    std::copy(result.x.begin(), result.x.end(), solution);
    statistics[0] = status;
    statistics[1] = result.total_cost;
    statistics[2] = result.n_func_evals;
    statistics[3] = result.n_qp_solves;
    statistics[4] = iteration;
    statistics[5] = optimizer.merit_error_coeff_;
    statistics[6] = optimizer.trust_box_size_;
    statistics[7] = result.cnt_viols.empty() ? 0. : result.cnt_viols[0];
    std::fflush(NULL);
    return 0;
  } catch (const std::exception& exception) {
    std::snprintf(error, error_size, "%s", exception.what());
    std::fflush(NULL);
    return -1;
  } catch (...) {
    std::snprintf(error, error_size, "Unknown native TrajOpt exception");
    std::fflush(NULL);
    return -2;
  }
}
