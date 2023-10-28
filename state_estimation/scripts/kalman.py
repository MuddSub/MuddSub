#! /usr/bin/python3
# This module defines Kalman-filter based state estimation classes
import numpy as np


def safe_inv(m: np.ndarray) -> np.ndarray:
    """Computes the inverse or pseudoinverse of a matrix, as necessary."""
    try:
        return np.linalg.inv(m)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(m)


class KalmanFilter:
    """An n-dimensional Kalman Filter"""

    def __init__(self, dim: int):
        """Creates a Kalman filter of the specified dimension."""
        self._z = np.zeros(dim)
        self._prec = np.zeros((dim, dim))
        self._x = np.zeros(dim)
        self._cov = None
    
    def step(self, timestep: float, vel: np.ndarray, cov: np.ndarray) -> None:
        """Increments the Kalman filter by one time step.
        
        Args:
            timestep: A float, the time increment of the step. (Required)
            vel: A float array specifying the velocity estimate of the
                state during the time step. (Required)
            cov: A float array specifying the covariance of the velocity
                estimate of the velocity during the time step. (Required)
        """
        self._cov = self.cov() + cov * timestep
        self._x = self.pred() + vel * timestep
        self._z = None
        self._prec = None
    
    def add(self, measurement: np.ndarray, prec: np.ndarray) -> None:
        """Updates the Kalman filter with a new measurement.
        
        Args:
            measurement: A float array, the new measurement of the state.
                (Required)
            prec: A float array, the precision matrix of the new measurement.
                (Requried)
        """
        z = prec @ measurement
        self._z = self.z() + z
        self._prec = self.prec() + prec
        self._x = None
        self._cov = None
    
    def pred(self) -> np.ndarray:
        """Returns the current estimate of the state."""
        if self._x is None:
            self._x = self.cov() @ self._z
        return self._x

    def cov(self) -> np.ndarray:
        """Returns the covariance of the current estimate of the state."""
        if self._cov is None:
            self._cov = safe_inv(self._prec)
        return self._cov
    
    def prec(self) -> np.ndarray:
        """Returns the precision of the current estimate of the state."""
        if self._prec is None:
            self._prec = safe_inv(self._cov)
        return self._prec

    def z(self) -> np.ndarray:
        """Returns the matrix-vector product of the current precision and state estimate."""
        if self._z is None:
            self._z = self.prec() @ self._x
        return self._z


class KinematicKalmanFilter:
    """An Extended Kalman filter tailored to simple kinematics."""

    def __init__(self, drift: np.ndarray, dim: int=None):
        """Creates a Kinematic Kalman filter.
        
        Args:
            drift: A float array representing the rate at which the
                covariance of the velocity accumulates per unit time.
                This may be a scalar, for an isotropic distribution,
                a vector where each element gives the covariance of
                a different component of the velocity, or a full
                covariance matrix. (Required)
            dim: If drift is a scalar, this specifies the dimensionality
                of the state and velocity vectors. Otherwise, this is
                ignored. (Default None)
        """
        dim = dim if dim is not None else drift.shape[-1]
        if isinstance(drift, np.ndarray) and len(drift.shape) == 2:
            self.drift = drift
        else:
            self.drift = np.eye(dim) * drift
        self.position_filter = KalmanFilter(dim)
        self.velocity_filter = KalmanFilter(dim)
    
    def step(self, timestep: float) -> None:
        """Increments the Kalman filter by one time step.
        
        Args:
            timestep: A float, the time increment of the step. (Required)
        """
        self.position_filter.step(timestep, self.velocity_filter.pred(), self.velocity_filter.cov())
        self.velocity_filter.step(timestep, np.zeros((self.drift.shape[-1],)), self.drift)
    
    def add_velocity(self, vel: np.ndarray, prec: np.ndarray) -> None:
        """Updates the Kalman filter with a new velocity measurement.
        
        Args:
            vel: A float array, the velocity measurement. (Required)
            prec: A float array, the precision matrix of the velocity
                measurement. (Required)
        """
        self.velocity_filter.add(vel, prec)
    
    def add_position(self, pos: np.ndarray, prec: np.ndarray) -> None:
        """Updates the Kalman filter with a new position measurement.
        
        Args:
            pos: A float array, the position measurement. (Required)
            prec: A float array, the precision matrix of the position
                measurement. (Required)
        """
        self.position_filter.add(pos, prec)
    
    def get_position(self) -> np.ndarray:
        """Returns the current estimate of the position."""
        return self.position_filter.pred()
    
    def get_velocity(self) -> np.ndarray:
        """Returns the current estimate of the velocity."""
        return self.velocity_filter.pred()
    
    def get_position_cov(self) -> np.ndarray:
        """Returns the covariance of the current estimate of the position."""
        return self.position_filter.cov()
    
    def get_position_prec(self) -> np.ndarray:
        """Returns the precision of the current estimate of the position."""
        return self.position_filter.prec()
    
    def get_velocity_cov(self) -> np.ndarray:
        """Returns the covariance of the current estimate of the velocity."""
        return self.velocity_filter.cov()
    
    def get_velocity_prec(self) -> np.ndarray:
        """Returns the precision of the current estimate of the velocity."""
        return self.velocity_filter.prec()
