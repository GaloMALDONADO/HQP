import numpy as np
import numpy.matlib
from pinocchio import SE3, log3, exp3, Motion
from derivative_filters import computeSecondOrderPolynomialFitting

''' Base class for a trajectory '''
class RefTrajectory (object):

  def __init__ (self, name):
    self._name = name
    self._dim = 0

  @property
  def dim(self):
    return self._dim

  def __call__ (self, t):
    return (np.matrix ([]).reshape (0, 0),)*3


''' An Nd trajectory with varying state and zero velocity/acceleration. 
    xref must be size a numpy matrix [dof, duration]
'''

class VaryingNdTrajectory (object):

  def __init__ (self, name, x_ref, dt):
    self._name = name
    self._dt = dt
    self._dim = x_ref.shape[0]
    self._x_ref = x_ref.copy()
    self._v_ref = self._x_ref*0
    self._a_ref = self._x_ref*0

  @property
  def dim(self):
    return self._dim
    
  def setReference(self, x_ref):
    assert x_ref.shape[0]==self._x_ref.shape[0]
    self._x_ref = x_ref

  def __call__ (self, t):
    assert t>=0.0, "Time must be non-negative"
    i = int(t/self._dt);
    if(i>=self._x_ref.shape[1]):
       raise ValueError("Specified time exceeds the duration of the trajectory: "+str(t))
    return (self._x_ref[:,i], self._v_ref[:,i], self._a_ref[:,i]);


''' An Nd trajectory with constant state and zero velocity/acceleration. '''
class ConstantNdTrajectory (object):

  def __init__ (self, name, x_ref):
    self._name = name
    self._dim = x_ref.shape[0]
    self._x_ref = np.matrix.copy(x_ref);
    self._v_ref = np.zeros(x_ref.shape);
    self._a_ref = np.zeros(x_ref.shape);

  @property
  def dim(self):
    return self._dim
    
  def setReference(self, x_ref):
    assert x_ref.shape[0]==self._x_ref.shape[0]
    self._x_ref = x_ref;

  def __call__ (self, t):
    return (self._x_ref, self._v_ref, self._a_ref);


    
''' An se3 trajectory with constant state and zero velocity/acceleration. '''
class ConstantSE3Trajectory (object):

  def __init__ (self, name, Mref):
    self._name = name
    self._dim = 6
    self._Mref = Mref;
    self._v_ref = Motion.Zero()
    self._a_ref = Motion.Zero()

  @property
  def dim(self):
    return self._dim
    
  def setReference(self, Mref):
    self._Mref = Mref;

  def __call__ (self, t):
    return (self._Mref, self._v_ref, self._a_ref);





''' An Nd trajectory computed from a specified discrete-time trajectory
    by applying a polynomial fitting. 
''' 
class SmoothedNdTrajectory (object):

  ''' Constructor.
      @param x_ref A NxT numpy matrix, where N is the size of the signal and T is the number of time steps
      @param dt The time step duration in seconds
      @param window_length An odd positive integer representing the size of the window used for the polynomial fitting
  '''
  def __init__ (self, name, x_ref, dt, window_length):
    self._name = name
    self._dim = x_ref.shape[0]
    self._dt  = dt;
    (self._x_ref, self._v_ref, self._a_ref) = computeSecondOrderPolynomialFitting(x_ref, dt, window_length);

  @property
  def dim(self):
    return self._dim

  def __call__ (self, t):
    assert t>=0.0, "Time must be non-negative"
    i = int(t/self._dt);
    if(i>=self._x_ref.shape[1]):
       raise ValueError("Specified time exceeds the duration of the trajectory: "+str(t))
    return (self._x_ref[:,i], self._v_ref[:,i], self._a_ref[:,i]);




''' An SE3 trajectory computed from a specified discrete-time trajectory
    by applying a polynomial fitting. 
''' 
class SmoothedSE3Trajectory (object):

  ''' Constructor.
      @param x_ref A list of T pinocchio.SE3 objects, where T is the number of time steps
      @param dt The time step duration in seconds
      @param window_length An odd positive integer representing the size of the window used for the polynomial fitting
  '''
  def __init__ (self, name, M_ref, dt, window_length):
    self._name = name;
    self._dim = 6;
    self._dt  = dt;
    self._M_ref = M_ref;
    x_ref = np.hstack([M.translation for M in M_ref]);
    (self._x_ref, self._v_ref, self._a_ref) = computeSecondOrderPolynomialFitting(x_ref, dt, window_length);

  @property
  def dim(self):
    return self._dim

  def __call__ (self, t):
    assert t>=0.0, "Time must be non-negative"
    i = int(t/self._dt);
    if(i>=self._x_ref.shape[1]):
       raise ValueError("Specified time exceeds the duration of the trajectory: "+str(t));
    M = self._M_ref[i];
    M.translation = self._x_ref[:,i];
    v = Motion.Zero();
    a = Motion.Zero();
    v.linear = self._v_ref[:,i];
    a.linear = self._a_ref[:,i];
    return (M, v, a);
