'''
Overview of MPC variables:
 Ad: a dynamic matrix of the robots state variables (i.e. center of gravity(velocity) + intertia) for each x,y,z,roll, pitch, yaw, and change in linear and angular velocity
 Bd: The 8 thrusters of the robot with vecotrized forces from each
 Q: The associated weights of each function to minimize error (i.e. how much weight error in z dimension (depth) has over error in y(range))
 Model constraints
 umin: max reverse force of thrusters in kg(f), minimum control effort
 umax: max forward force of thrusters in kg(f), maximum control effort
 xmin: minimum output for all states in order of roll pitch yaw x y z, and velocities
 xmax: maximum otuput for all states in order of roll pitch yaw x y z, and velocities
 nsim: number of times model optimizes
 x0: plantState from slam (pose)
 xr: setState from trajectory
 N: prediction horizon - number of steps MPC looks into the future
 P: diagnal matrix with weights of model (Q), entries of the minimized error matrix Q repeated (N+1) times and 0.1 (4*N) times.
 Matrix Math Overview:
 Intermediate Q stacks arrays of -Q.(xr), and adds zeros at the end
 Ax krons a matrix of 1s and 0s with Ad, our model of our path plan from navigation
 Bu stacks a bunch of matrics and krons the result with Bd, our future prediction
 Aeq hstacked Ax and Bu
 Main function overview:
 loop
 res = prob.solve()
 ctrl = res.x[-N*nu:-(N-1)*nu]
 x0 = Ad.dot(x0) + Bd.dot(ctrl), updates current position
 updates position and control effort into our initial states for mpc
ROS Subscribers and Nodes:
 #Inputs
 self.trajectorySub = rospy.Subscriber("/controls/trajectory", Path, self.trajectoryCB) #navigation's trajectory
 self.poseSub = rospy.Subscriber("slam/robot/pose", Odometry, self.poseCB) #ask if msg type slam is /pose or /state
 #Outputs
 self.wrenchPub = rospy.Publisher("/controls/robot/wrench",geometry_msgs.msg.WrenchStamped, queue_size= 1) #total thruster output force and torque

 Compute Matrix         0
         0
  -6094.16
         0
         0
         0
         0
         0
3.0471e+06
         0
         0
         0       0
       0
 12.1881
       0
       0
       0
       0
       0
-6094.16
       0
       0
       01.61757e+09
[ INFO] [1617572548.096853897]: Time: -1.001985
[ INFO] [1617572549.085581283]: Iterating simulator
STATE: 
       0
       0
 12.1881
       0
       0
       0
       0
       0
-6094.16
       0
       0
       0
DERIV: 
         0
         0
  -6094.16
         0
         0
         0
         0
         0
3.0471e+06
         0
         0
         0

roscore 
rosrun xacro xacro ../../core/descriptions/alfie.urdf.xacro -o ../../core/descriptions/alfie.urdf 
rosrun controls simulation_dynamics
'''
import numpy as np
import scipy as sp
from scipy import sparse
import osqp #matrix optimizer
import scripts/vehicleDynamics
"""
//https://github.com/MuddSub/MuddSub/blob/develop/controls/controls/scripts/VehicleDynamics.py
//read in yaml file: controls/cfg/dynamics.yaml from ros?
//code review ask what rate means
//linearizedSystem = control.linearize(self.nonlinearIoSys, self.x, self.u)
//dt = a/self.rate
Taken from: https://github.com/MuddSub/MuddSub/blob/develop/base/controls/cfg/dynamics.yaml
"""
rate = 50
dt = 1/rate
mass = 20 #kg
MPC_matrix = VehicleDynamics();
#A temporary matrix of the correct dimensions of models dynamics


Ad = sparse.csc_matrix([ #Figure out wtf this matrix is next week
  [0, 0, 12.2132, 0, 0, 0],
  [ 0, 0, -6106.65, 0, 0, 0],
  [0, 0, -0.0245242, 0, 0, 0]
  [0, 0, 12.2132, 0, 0, 01.61757e+09]
])

Ad = sparse.csc_matrix([
  [1.,      0.,     0., 0., 0., 0., dt,     0.,     0.,  0.,      0.,     0.    ],
  [0.,      1.,     0., 0., 0., 0., 0.,      dt,    0.,  0.,      0.,     0.    ],
  [0.,      0.,     1., 0., 0., 0., 0.,      0.,     dt, 0.,      0.,     0.    ],
  [0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  dt,     0.,     0.    ],
  [0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     dt,     0.    ],
  [0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     dt    ],
  [0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ],
  [0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  1.,     0.,     0.    ],
  [0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     1.,     0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     1.    ]
])
#vectorized thruster forces, each column is a thruster
#Wrench x y z r p y
Bd = sparse.csc_matrix([
  [0.,              0.,              0.,              ((dt**2)/(2*mass)), 0.,              0.             ], #roll
  [0.,              0.,              0.,              0.,              (dt**2)/(2*mass), 0.             ], #pitch
  [0.,              0.,              0.,              0.,              0.,              (dt**2)/(2*mass)], #yaw
  [(dt**2)/(2*mass), 0.,              0.,              0.,              0.,              0.             ], #x
  [0.,              (dt**2)/(2*mass), 0.,              0.,              0.,              0.             ], #y
  [0.,              0.,              (dt**2)/(2*mass), 0.,              0.,              0.             ], #z
  [0.,              0.,              0.,              dt/mass,         0.,              0.             ], #roll'
  [0.,              0.,              0.,              0.,              dt/mass,         0.             ], #pitch'
  [0.,              0.,              0.,              0.,              0.,              dt/mass        ], #yaw'
  [dt/mass,         0.,              0.,              0.,              0.,              0.             ], #x'
  [0.,              dt/mass,         0.,              0.,              0.,              0.             ], #y'
  [0.,              0.,              dt/mass,         0.,              0.,              0.             ]])#z'
[nx, nu] = Bd.shape #nx is number of states, nu is number of thrusters
#Constraints in kgf = 9.81kgN
umin = np.array([-4.1, -4.1, -4.1, -4.1, -4.1, -4.1]) #reverse thruster power
umax = np.array([5.25, 5.25, 5.25, 5.25, 5.25, 5.25]) #forward thruster power
xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,0.1,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf]) #[roll pitch yaw x y z, and linear and angular velocities]
xmax = np.array([ np.inf, np.inf, np.inf, np.inf, np.inf, 10,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]) #[roll pitch yaw x y z, and linear and angular velocities]
#how much we care about being off of our target state for each state variable
#roll pitch yaw x y z and their respective velocities
Q = sparse.diags([100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100])
QN = Q
R = sparse.eye(6)
# initial comes from slam and reference comes from navigation
#TEMPORARY
x0 = np.zeros(12)#input from slam plant state
xr = np.array([0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]) #input from trajectory
# Prediction horizon. how many steps ahead we are looking
N = 10
# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)], format='csc')
# - linear objective
q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
               np.zeros(N*nu)])
# - linear dynamics
Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(N*nx)])
ueq = leq
# - input and state constraints
Aineq = sparse.eye((N+1)*nx + N*nu)
lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
# - OSQP constraints
A = sparse.vstack([Aeq, Aineq], format='csc')
l = np.hstack([leq, lineq])
u = np.hstack([ueq, uineq])
# Create an OSQP object
prob = osqp.OSQP()
# Setup workspace
prob.setup(P, q, A, l, u, warm_start=True)
# Simulate in closed loop
nsim = 15
for i in range(nsim):
    # Solve
    res = prob.solve()
    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')
    # Apply first control input to the plant
    ctrl = res.x[-N*nu:-(N-1)*nu]
    x0 = Ad.dot(x0) + Bd.dot(ctrl)
    x0[0] = x0[0] % (2 * np.pi)
    x0[1] = x0[1] % (2 * np.pi)
    x0[2] = x0[2] % (2 * np.pi)
    # Update initial state
    l[:nx] = -x0
    u[:nx] = -x0
    prob.update(l=l, u=u)
    print(i)
