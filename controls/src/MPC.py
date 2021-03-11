'''
 TODO: give examples of how this works for us

 initialize Ad path plan from navigation
 initialize Bd Model of our future prediction
 nx and nu are dimensions of Bd
 Q and QN are diagonal matrices
 objective function is the integral of the error between navigation and future prediction
 ?(R is 0.1*I^4)

 constraints
 umin: minimum control effort
 umax: maximum control effort
 xmin: minimum x value (array for all states)
 xmax: maximum x value (array for all states)

 #Inputs
 self.trajectorySub = rospy.Subscriber("/controls/trajectory", Path, self.trajectoryCB) #navigation's trajectory
 self.poseSub = rospy.Subscriber("slam/robot/pose", Odometry, self.poseCB) #ask if msg type slam is /pose or /state

 #Outputs
 self.wrenchPub = rospy.Publisher("/controls/robot/wrench",geometry_msgs.msg.WrenchStamped, queue_size= 1) #total thruster output force and torque

 x0 is the initial state in (x, y, z, roll, pitch, yaw, x', y', z', roll', pitch', yaw')

 xr is the reference state in the same form

 N is the prediction horizon: 10

 P is a diagonal matrix with the diagonal entries of the minimized error matrix Q repeated
 (N+1) times and 0.1 (4*N) times.
 Q stacks arrays of -Q.(xr), and adds zeros at the end
 Ax krons a matrix of 1s and 0s with Ad, our model of our path plan from navigation
 Bu stacks a bunch of matrics and krons the result with Bd, our future prediction
 Aeq hstacked Ax and Bu

 nsim is the number of times we optimize, while N is how far into the future we look



 loop
 res = prob.solve()
 ctrl = res.x[-N*nu:-(N-1)*nu]
 x0 = Ad.dot(x0) + Bd.dot(ctrl), updates current position
 updates position and control effort into our initial states for mpc
'''


import numpy as np
import scipy as sp
from scipy import sparse
import osqp
dt = 0.1

#we need to figure this out better
#(roll, pitch, yaw, x, y, z, and all velocities)
#CG_x = CG(1);
# CG_y = CG(2);
# CG_z = CG(3);
# C_12 = m* [(CG_y*velocity_q + CG_z*velocity_r), -(CG_x*velocity_q - velocity_w), -(CG_x*velocity_r + velocity_v);
# -(CG_y*velocity_p + velocity_w) , (CG_z*velocity_r + CG_x*velocity_p), -(CG_y*velocity_r - velocity_u);
# -(CG_z*velocity_p - velocity_v), -(CG_z*velocity_q + velocity_u), (CG_x*velocity_p+CG_y*velocity_q)];
# C_22 = [0, -Iyz*velocity_q-Ixz*velocity_p+Izz*velocity_r, Iyz*velocity_r+Ixy*velocity_p-Iyy*velocity_q;
# Iyz*velocity_q+Ixz*velocity_p-Izz*velocity_r, 0, -Ixz*velocity_r-Ixy*velocity_q+Ixx*velocity_p;
# -Iyz*velocity_r-Ixy*velocity_p-Iyy*velocity_q, Ixz*velocity_r+Ixy*velocity_q-Ixx*velocity_p, 0];
#
#
# C_a = ...
# [ 0, 0, 0, 0, -M_aZ*velocity_w, M_aY*velocity_v;
# 0, 0, 0, M_aZ*velocity_w, 0, -M_aX*velocity_u;
# 0, 0, 0, -M_aY*velocity_v, -M_aX*velocity_u, 0;
# 0, -M_aZ*velocity_w, -M_aY*velocity_v, 0, -M_ay*velocity_r, -M_ap*velocity_q;
# M_aZ*velocity_w, 0, -M_aX*velocity_u, -M_ay*velocity_
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

#Control dynamics matrix. Get from Max. Columns are thrusters, rows are state variables.
Bd = sparse.csc_matrix([
  [0.,      -0.0726,  0.,     0.0726,0.,      -0.0726,  0.,     0.0726],
  [-0.0726,  0.,      0.0726, 0.    ,-0.0726,  0.,      0.0726, 0.    ],
  [-0.0152,  0.0152, -0.0152, 0.0152,-0.0152,  0.0152, -0.0152, 0.0152],
  [-0.,     -0.0006, -0.,     0.0006,-0.,     -0.0006, -0.,     0.0006],
  [0.0006,   0.,     -0.0006, 0.0000,0.0006,   0.,     -0.0006, 0.0000],
  [0.0106,   0.0106,  0.0106, 0.0106,0.0106,   0.0106,  0.0106, 0.0106],
  [0,       -1.4512,  0.,     1.4512,0,       -1.4512,  0.,     1.4512],
  [-1.4512,  0.,      1.4512, 0.    ,-1.4512,  0.,      1.4512, 0.    ],
  [-0.3049,  0.3049, -0.3049, 0.3049,-0.3049,  0.3049, -0.3049, 0.3049],
  [-0.,     -0.0236,  0.,     0.0236,-0.,     -0.0236,  0.,     0.0236],
  [0.0236,   0.,     -0.0236, 0.    ,0.0236,   0.,     -0.0236, 0.    ],
  [0.2107,   0.2107,  0.2107, 0.2107,0.2107,   0.2107,  0.2107, 0.2107]])
#nx is number of states, nu is number of thrusters
[nx, nu] = Bd.shape

#Constraints in kgf = 9.81kgN
umin = np.array([-4.1, -4.1, -4.1, -4.1, -4.1, -4.1, -4.1, -4.1])
umax = np.array([5.25, 5.25, 5.25, 5.25, 5.25, 5.25, 5.25, 5.25])

#roll pitch yaw x y z, and velocities
xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,0.1,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, 10,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

#how much we care about being off of our target state for each state variable
#roll pitch yaw x y z and their respective velocities
Q = sparse.diags([1, 1, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10])
QN = Q
R = 0.1*sparse.eye(8)

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

    # Update initial state
    l[:nx] = -x0
    u[:nx] = -x0
    prob.update(l=l, u=u)
    print(i)
print("hello")
