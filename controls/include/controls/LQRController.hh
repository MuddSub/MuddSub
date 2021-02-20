#pragma once

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "controls/Types.hh"

namespace MuddSub::Controls
{

/** @brief LQRController is a simple wrapper for the control toolbox LQR solver.

This class implements an LQR solver by modifying the behavior of control toolbox's
LQR class such that computeControl() solves the control problem,
and returns the result. Hence, the interface more closely resembles a class
derived from ct::core::Controller.

It also provides a ros-friendly tuning interface.
*/
template<size_t lqrStateDim, size_t lqrControlDim>
class LQRController
{


public:
  /// A matrix is (lqrStateDim x lqrStateDim)
  using AMatrix_t = Eigen::Matrix<double, lqrStateDim, lqrStateDim>;

  /// B matrix is (lqrStateDim x lqrControlDim)
  using BMatrix_t = Eigen::Matrix<double, lqrStateDim, lqrControlDim>;

  /// Q matrix controls state error, so it's (lqrStateDim x lqrStateDim)
  using QMatrix_t = Eigen::Matrix<double, lqrStateDim, lqrStateDim>;

  /// R Matrix minimizes control effort, and is thus (lqrControlDim x lqrControlDim)
  using RMatrix_t = Eigen::Matrix<double, lqrControlDim, lqrControlDim>;

  /// K Matrix is full-state-feedback gains, and is (lqrControlDim x lqrControlDim)
  using KMatrix_t = Eigen::Matrix<double, lqrControlDim, lqrStateDim>;

  /// State vector for LQR (usually 8-vector)
  using lqrStateVector_t = ct::core::StateVector<lqrStateDim, double>;

  /// The control vector for LQR (usually 4-vector)
  using lqrControlVector_t = ct::core::ControlVector<lqrControlDim, double>;

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
  lqrControlVector_t computeControl(const AMatrix_t& A, const BMatrix_t& B,
                                  const lqrStateVector_t& controlAction);


  /// @Param: Q: state error cost matrix to set
  inline void setQ(const QMatrix_t Q)
  {
    Q_ = Q;
  };

  /// @Param: R: Effort cost matrix to set
  inline void setR(const QMatrix_t R)
  {
    R_ = R;
  };

  /// Sets the error cost matrix to diagonal with each entry equal to q
  /// @param q: value for each entry on the diagonal of the cost matrix
  inline void setQUniformDiag(const double& q)
  {
    Q_ = QMatrix_t::Identity() * q;
  }

  /// Sets the effort cost matrix to diagonal with each entry equal to r
  /// @param r: value for each entry on the diagonal of the cost matrix
  inline void setRUniformDiag(const double& r)
  {
    R_ = QMatrix_t::Identity() * r;
  }

private:
  /// Solver for optimal control problem
  ct::optcon::LQR<lqrStateDim, lqrControlDim> lqrSolver_;

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
