/// @file ocp_nlp.cpp
/// @brief C++ wrapper implementation for Acados OCP NLP interface
/// @author Pedram Rabiee (prabiee@3laws.io)
/// @copyright Copyright 2025

#include "acados_cpp/ocp_nlp.hpp"

#include <stdexcept>
#include <cstring>

// Include Acados C headers
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
}

namespace acados {

OcpNlpSolver::OcpNlpSolver(const OcpNlpDims& dims, const OcpNlpConfig& config)
    : dims_(dims), config_(config)
{
  // TODO: Initialize Acados structures
  // This is a placeholder implementation
  // We'll implement this properly after understanding the C API better

  if (dims_.nx <= 0 || dims_.nu <= 0 || dims_.N <= 0) {
    throw std::invalid_argument("Invalid dimensions: nx, nu, and N must be positive");
  }

  // Placeholder: We'll implement full initialization later
  // For now, just validate dimensions
}

OcpNlpSolver::~OcpNlpSolver()
{
  // TODO: Free Acados structures
  // ocp_nlp_solver_destroy(nlp_solver_);
  // ocp_nlp_config_free(nlp_config_);
  // etc.
}

void OcpNlpSolver::set_initial_state(const std::vector<double>& x0)
{
  if (static_cast<int>(x0.size()) != dims_.nx) {
    throw std::invalid_argument("x0 size must match nx");
  }

  // TODO: Call Acados C API
  // ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x0.data());
  // ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x0.data());
}

void OcpNlpSolver::set_state_reference(int stage, const std::vector<double>& x_ref)
{
  if (stage < 0 || stage > dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  if (static_cast<int>(x_ref.size()) != dims_.nx) {
    throw std::invalid_argument("x_ref size must match nx");
  }

  // TODO: Call Acados C API
  // ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref", x_ref.data());
}

void OcpNlpSolver::set_control_reference(int stage, const std::vector<double>& u_ref)
{
  if (stage < 0 || stage >= dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  if (static_cast<int>(u_ref.size()) != dims_.nu) {
    throw std::invalid_argument("u_ref size must match nu");
  }

  // TODO: Call Acados C API
}

void OcpNlpSolver::set_cost_Q(int stage, const std::vector<double>& Q)
{
  if (stage < 0 || stage > dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  if (static_cast<int>(Q.size()) != dims_.nx * dims_.nx) {
    throw std::invalid_argument("Q size must be nx*nx");
  }

  // TODO: Call Acados C API
  // ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "W", Q.data());
}

void OcpNlpSolver::set_cost_R(int stage, const std::vector<double>& R)
{
  if (stage < 0 || stage >= dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  if (static_cast<int>(R.size()) != dims_.nu * dims_.nu) {
    throw std::invalid_argument("R size must be nu*nu");
  }

  // TODO: Call Acados C API
}

int OcpNlpSolver::solve()
{
  // TODO: Call Acados solver
  // int status = ocp_nlp_solve(nlp_solver_, nlp_in_, nlp_out_);
  // return status;

  // Placeholder: return success
  return 0;
}

std::vector<double> OcpNlpSolver::get_state(int stage) const
{
  if (stage < 0 || stage > dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  std::vector<double> x(dims_.nx, 0.0);

  // TODO: Call Acados C API
  // ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

  return x;
}

std::vector<double> OcpNlpSolver::get_control(int stage) const
{
  if (stage < 0 || stage >= dims_.N) {
    throw std::out_of_range("Stage index out of range");
  }

  std::vector<double> u(dims_.nu, 0.0);

  // TODO: Call Acados C API
  // ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());

  return u;
}

int OcpNlpSolver::get_iterations() const
{
  // TODO: Get from Acados
  // int iter;
  // ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &iter);
  // return iter;

  return 0;
}

double OcpNlpSolver::get_solve_time() const
{
  // TODO: Get from Acados
  // double time;
  // ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &time);
  // return time;

  return 0.0;
}

double OcpNlpSolver::get_cost() const
{
  // TODO: Get from Acados
  // double cost;
  // ocp_nlp_eval_cost(nlp_solver_, nlp_in_, nlp_out_, &cost);
  // return cost;

  return 0.0;
}

}  // namespace acados
