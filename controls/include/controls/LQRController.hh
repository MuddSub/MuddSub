#pragma once

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

namespace MuddSub::Controls
{

/** @brief LQRController is a simple wrapper for the control toolbox LQR solver.

This class implements an LQR solver by modifying the behavior of control toolbox's
LQR class such that computeControl() solves the control problem,
and returns the result. Hence, the interface more closely resembles a class
derived from ct::core::Controller.

It also provides a ros-friendly tuning interface.
*/
template<size_t stateDim, size_t controlDim>
class LQRController
{


public:

  using stateVector_t = ct::core::StateVector<stateDim, double>;
  using controlVector_t = ct::core::ControlVector<controlDim, double>;

  /// A matrix is (stateDim x stateDim)
  using AMatrix_t = Eigen::Matrix<double, stateDim, stateDim>;

  /// B matrix is (stateDim x controlDim)
  using BMatrix_t = Eigen::Matrix<double, stateDim, controlDim>;

  /// Q matrix controls state error, so it's (stateDim x stateDim)
  using QMatrix_t = Eigen::Matrix<double, stateDim, stateDim>;

  /// R Matrix minimizes control effort, and is thus (controlDim x controlDim)
  using RMatrix_t = Eigen::Matrix<double, controlDim, controlDim>;

  /// K Matrix is full-state-feedback gains, and is (controlDim x controlDim)
  using KMatrix_t = Eigen::Matrix<double, controlDim, stateDim>;

  /// The default constructor sets Q to identity, and R to .001*Identity
  LQRController();

  /// @param qCoefficient sets all diagonal coefficients of Q to qCoefficient
  /// @param rCoefficient sets all diagonal coefficients of the R matrix to rCoefficient
  LQRController(const double& qCoefficient, const double& rCoefficient);

  /// @param Q: the desired state error cost matrix
  /// @param R: the desired control effort cost matrix
  LQRController(const QMatrix_t& Q, const RMatrix_t& R);

  /// Given linear A and B matrices, compute the optimal control by solving the
  /// LQR problem.
  /// @param A: Linearized matrix specifying how state derivative depends on curent state
  /// @param B: Linearized matrix specifying how state derivative depends on input
  /// @returns controlAction: Optimal control action.
  controlVector_t computeControl(const AMatrix_t& A, const BMatrix_t& B,
                                  const stateVector_t& controlAction);


private:
  /// Solver for optimal control problem
  ct::optcon::LQR<stateDim, controlDim> lqrSolver_;

  /// Feedback matrix: result of LQR solve.
  KMatrix_t K_;

  /// LQR weights for error
  QMatrix_t Q_{QMatrix_t::Identity()};

  /// LQR weights for control effort
  /// \pre This is assumed diagonal for efficiency.
  RMatrix_t R_{RMatrix_t::Identity()};
};
}

#include "controls/LQRController.inl"
