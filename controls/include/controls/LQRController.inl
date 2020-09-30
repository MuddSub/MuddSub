namespace MuddSub::Controls
{

template<size_t stateDim, size_t controlDim>
LQRController<stateDim, controlDim>::LQRController()
{
  // Q_ and R_ start as identity; by default, just make R nice and small
  Q_ *= 100;
}

template<size_t stateDim, size_t controlDim>
LQRController<stateDim, controlDim>::LQRController(const double& qCoefficient,
                                                   const double& rCoefficient)
{
  // Q_ and R_ start as identity, scale as desired
  R_ *= rCoefficient;
  Q_ *= qCoefficient;
}

template<size_t stateDim, size_t controlDim>
LQRController<stateDim, controlDim>::LQRController(const QMatrix_t& Q,
                                                   const RMatrix_t& R):
                                                   Q_{Q}, R_{R}
{
}


// Compute the optimal control
template<size_t stateDim, size_t controlDim>
typename LQRController<stateDim, controlDim>::controlVector_t
                            LQRController<stateDim, controlDim>::computeControl(
                                        const AMatrix_t& A, const BMatrix_t& B,
                                        const stateVector_t& error)
{
  // Explicitly solves ARE. The 'true' option specifies r to be diagonal
  lqrSolver_.compute(Q_, R_, A, B, K_, true);
  return -1*K_*error;
}
}
