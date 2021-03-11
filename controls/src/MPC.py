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
'''

import numpy as np
import scipy as sp
from scipy import sparse
import osqp #matrix optimizer

"""
#TODO: change to real values of robot, current guesses
dt = 0.1 #change in time between steps
robot_mass = 52 #kg, replace with real weight
CG_x, CG_y, CG_z = 0,0, -.1
vel_x, vel_y, vel_z, vel_r, vel_p, vel_yaw = 1,0,0,0,0,0 #change to input from dvl?
Ixx, Ixy, Ixz, Iyy, Iyz, Izz = 1,1,.5,0,1,.25 #ake inertia adjustable
M_aX, M_aY, M_aZ, M_ar, M_ap, M_ay = 1,.25,0,0,0,0 #addedc mass coef

C_CG = robot_mass* [(CG_y*vel_p + CG_z*vel_yaw), -(CG_x*vel_p - vel_z), -(CG_x*vel_yaw + vel_y),
          -(CG_y*vel_r + vel_z) , (CG_z*vel_yaw + CG_x*vel_r), -(CG_y*vel_yaw - vel_x),
          -(CG_z*vel_r - vel_y), -(CG_z*vel_p + vel_x), (CG_x*vel_r+CG_y*vel_p)]

C_Interia = [0, -Iyz*vel_p-Ixz*vel_r+Izz*vel_yaw, Iyz*vel_yaw+Ixy*vel_r-Iyy*vel_p,
           Iyz*vel_p+Ixz*vel_r-Izz*vel_yaw, 0, -Ixz*vel_yaw-Ixy*vel_p+Ixx*vel_r,
           -Iyz*vel_yaw-Ixy*vel_r-Iyy*vel_p, Ixz*vel_yaw+Ixy*vel_p-Ixx*vel_r, 0]          

C_a = [ 0, 0, 0, 0, -M_aZ*vel_z, M_aY*vel_y,
        0, 0, 0, M_aZ*vel_z, 0, -M_aX*vel_x,
        0, 0, 0, -M_aY*vel_y, -M_aX*vel_x, 0,
        0, -M_aZ*vel_z, -M_aY*vel_y, 0, -M_ay*vel_yaw, -M_ap*vel_p,
        M_aZ*vel_z, 0, -M_aX*vel_x, -M_ay*vel_yaw, 0, -M_ar*vel_r,
        -M_aY*vel_y, M_aX*vel_x, 0, -M_ap*vel_p, -M_ar * vel_r, 0]

C_rb = [np.zeros(3), C_CG, -np.transpose(C_CG), C_Interia]
    
Ad = sparse.csc_matrix((C_a + C_rb)) #fix sparese matrix (maybe numpy add?)
"""

#A temporary matrix of the correct dimensions of models dynamics
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

[nx, nu] = Bd.shape #nx is number of states, nu is number of thrusters

#Constraints in kgf = 9.81kgN
umin = np.array([-4.1, -4.1, -4.1, -4.1, -4.1, -4.1, -4.1, -4.1]) #reverse thruster power
umax = np.array([5.25, 5.25, 5.25, 5.25, 5.25, 5.25, 5.25, 5.25]) #forward thruster power
xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,0.1,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf]) #[roll pitch yaw x y z, and linear and angular velocities]
xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, 10,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]) #[roll pitch yaw x y z, and linear and angular velocities]

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
    x0 = Ad.dopip
    
    t(x0) + Bd.dot(ctrl)

    # Update initial state
    l[:nx] = -x0
    u[:nx] = -x0
    prob.update(l=l, u=u)
    print(i)

