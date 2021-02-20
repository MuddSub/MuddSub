namespace MuddSub::Controls
{

template<size_t lqrStateDim, size_t lqrControlDim>
LQRController<lqrStateDim, lqrControlDim>::LQRController()
{
}

template<size_t lqrStateDim, size_t lqrControlDim>
LQRController<lqrStateDim, lqrControlDim>::LQRController(const double& qCoefficient,
                                                   const double& rCoefficient)
{
  // Q_ and R_ start as identity, scale as desired
  R_ *= rCoefficient;
  Q_ *= qCoefficient;
}

template<size_t lqrStateDim, size_t lqrControlDim>
LQRController<lqrStateDim, lqrControlDim>::LQRController(const QMatrix_t& Q,
                                                   const RMatrix_t& R):
                                                   Q_{Q}, R_{R}
{
}


// Compute the optimal control
template<size_t lqrStateDim, size_t lqrControlDim>
typename LQRController<lqrStateDim, lqrControlDim>::lqrControlVector_t
                            LQRController<lqrStateDim, lqrControlDim>::computeControl(
                                        const AMatrix_t& A, const BMatrix_t& B,
                                        const lqrStateVector_t& error)
{
  // Explicitly solves ARE. The 'true' option specifies r to be diagonal
  lqrSolver_.compute(Q_, R_, A, B, K_, true);
  return -1*K_*error;
}
}
