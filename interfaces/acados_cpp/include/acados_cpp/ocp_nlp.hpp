/// @file ocp_nlp.hpp
/// @brief C++ wrapper for Acados OCP NLP interface
/// @author Pedram Rabiee (prabiee@3laws.io)
/// @copyright Copyright 2025

#ifndef ACADOS_CPP_OCP_NLP_HPP
#define ACADOS_CPP_OCP_NLP_HPP

#include <memory>
#include <string>
#include <vector>

// Forward declare C types
extern "C" {
struct ocp_nlp_config;
struct ocp_nlp_dims;
struct ocp_nlp_in;
struct ocp_nlp_out;
struct ocp_nlp_solver;
struct ocp_nlp_plan;
struct ocp_nlp_opts;
}

namespace acados {

/// @brief OCP NLP solver configuration
struct OcpNlpConfig {
  /// @brief NLP solver type
  enum class NlpSolverType {
    SQP,      ///< Sequential Quadratic Programming
    SQP_RTI   ///< SQP with Real-Time Iteration
  };

  /// @brief QP solver type
  enum class QpSolverType {
    PARTIAL_CONDENSING_HPIPM,  ///< Partial condensing HPIPM
    FULL_CONDENSING_QPOASES,   ///< Full condensing qpOASES
    FULL_CONDENSING_HPIPM      ///< Full condensing HPIPM
  };

  /// @brief Integrator type
  enum class IntegratorType {
    ERK,   ///< Explicit Runge-Kutta
    IRK,   ///< Implicit Runge-Kutta
    GNSF,  ///< Generalized Nonlinear Static Feedback
    DISCRETE  ///< Discrete dynamics
  };

  NlpSolverType nlp_solver = NlpSolverType::SQP;
  QpSolverType qp_solver = QpSolverType::PARTIAL_CONDENSING_HPIPM;
  IntegratorType integrator = IntegratorType::ERK;

  int max_iter = 100;
  double tol = 1e-6;
};

/// @brief OCP NLP problem dimensions
struct OcpNlpDims {
  int nx = 0;  ///< Number of states
  int nu = 0;  ///< Number of inputs
  int nz = 0;  ///< Number of algebraic variables
  int np = 0;  ///< Number of parameters
  int N = 0;   ///< Number of shooting intervals (horizon)
};

/// @brief OCP NLP solver class
/// @details RAII wrapper around Acados C API for OCP NLP problems
class OcpNlpSolver {
public:
  /// @brief Constructor
  /// @param dims Problem dimensions
  /// @param config Solver configuration
  OcpNlpSolver(const OcpNlpDims& dims, const OcpNlpConfig& config = OcpNlpConfig{});

  /// @brief Destructor - cleans up Acados resources
  ~OcpNlpSolver();

  // Delete copy/move constructors (manage C resources)
  OcpNlpSolver(const OcpNlpSolver&) = delete;
  OcpNlpSolver& operator=(const OcpNlpSolver&) = delete;
  OcpNlpSolver(OcpNlpSolver&&) = delete;
  OcpNlpSolver& operator=(OcpNlpSolver&&) = delete;

  /// @brief Set initial state
  /// @param x0 Initial state (size nx)
  void set_initial_state(const std::vector<double>& x0);

  /// @brief Set state reference at stage
  /// @param stage Stage index (0 to N)
  /// @param x_ref State reference (size nx)
  void set_state_reference(int stage, const std::vector<double>& x_ref);

  /// @brief Set control reference at stage
  /// @param stage Stage index (0 to N-1)
  /// @param u_ref Control reference (size nu)
  void set_control_reference(int stage, const std::vector<double>& u_ref);

  /// @brief Set state cost weight matrix at stage
  /// @param stage Stage index (0 to N)
  /// @param Q Weight matrix (size nx*nx, column-major)
  void set_cost_Q(int stage, const std::vector<double>& Q);

  /// @brief Set control cost weight matrix at stage
  /// @param stage Stage index (0 to N-1)
  /// @param R Weight matrix (size nu*nu, column-major)
  void set_cost_R(int stage, const std::vector<double>& R);

  /// @brief Solve the OCP
  /// @return Solver status (0 = success)
  int solve();

  /// @brief Get state solution at stage
  /// @param stage Stage index (0 to N)
  /// @return State vector (size nx)
  std::vector<double> get_state(int stage) const;

  /// @brief Get control solution at stage
  /// @param stage Stage index (0 to N-1)
  /// @return Control vector (size nu)
  std::vector<double> get_control(int stage) const;

  /// @brief Get solver statistics
  /// @return Solver iterations used
  int get_iterations() const;

  /// @brief Get solve time
  /// @return Solve time in seconds
  double get_solve_time() const;

  /// @brief Get cost value
  /// @return Objective function value
  double get_cost() const;

  /// @brief Get dimensions
  const OcpNlpDims& get_dims() const { return dims_; }

private:
  OcpNlpDims dims_;
  OcpNlpConfig config_;

  // Acados C structures (opaque pointers)
  ocp_nlp_config* nlp_config_ = nullptr;
  ocp_nlp_dims* nlp_dims_ = nullptr;
  ocp_nlp_in* nlp_in_ = nullptr;
  ocp_nlp_out* nlp_out_ = nullptr;
  ocp_nlp_solver* nlp_solver_ = nullptr;
  ocp_nlp_plan* nlp_plan_ = nullptr;
  ocp_nlp_opts* nlp_opts_ = nullptr;

  void* capsule_ = nullptr;  // Acados capsule for memory management
};

}  // namespace acados

#endif  // ACADOS_CPP_OCP_NLP_HPP
