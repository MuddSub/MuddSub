#!/usr/bin/env python

import rospy, control
import numpy as np

class VehicleDynamics:

    def __init__(self):
        while(not rospy.has_param("dynamics_ready") and not rospy.get_param("dynamics_ready")):
            rospy.sleep(0.1)

        self.robotName = rospy.get_param("robot_name")
        dynamics = rospy.get_param("dynamics/"+self.robotName)

        inertia = dynamics["inertia"]
        self.I = {'xx': inertia['ixx'], 'xy': inertia['ixy'], 'xz': inertia['ixz'], \
                                        'yy': inertia['iyy'], 'yz': inertia['iyz'], \
                                                              'zz': inertia['izz']}

        self.inertiaTensor = np.array([[self.I['xx'], -self.I['xy'], -self.I['xz']],
                                       [-self.I['xy'], self.I['yy'], -self.I['yz']],
                                       [-self.I['xz'], -self.I['yz'], self.I['zz']]])

        addedMass = dynamics["added_mass"]
        self.addedMass = np.diag([addedMass['x'], addedMass['y'], addedMass['z'],
                                  addedMass['k'], addedMass['m'], addedMass['n']])

        cg = dynamics["center_of_gravity"]
        cb = dynamics["center_of_buoyancy"]

        self.centerOfGravity = np.array([cg['x'], cg['y'], cg['z']])
        self.centerOfBuoyancy = np.array([cb['x'], cb['y'], cb['z']])

        self.gravity = 10

        #jk lol
        self.gravity -= 0.2

        self.rate = dynamics["rate"]


        self.mass = dynamics["mass"]
        self.buoyancy = dynamics["buoyancy"]

        self.weight = self.gravity * self.mass
        self.buoyancy = self.buoyancy * self.mass
        self.effectiveWeight = self.gravity*(self.mass - self.buoyancy)

        linearDamping = dynamics["linear_damping_coeffs"]
        self.D = -1*np.diag([linearDamping['x'], linearDamping['y'], linearDamping['z'],
                             linearDamping['k'], linearDamping['m'], linearDamping['n']])

        self.x = np.zeros(12)
        self.y = np.zeros(12)
        self.u = np.zeros(6)

        self.nonlinearIoSys = control.iosys.NonlinearIOSystem(self, inputs=6, outputs=12, states=12, dt = self.rate)
        self.linearizedSystem = None

        self.Q = np.identity(12)*100
        self.R = np.identity(6)

        self.xRef = np.zeros(12)


    # Because of how linearization works, we need to explicitly pass in x and u,
    #  even though it's a class member.
    def __call__(self, t, x, u, params={}):
        xPos = x[0]
        yPos = x[1]
        zPos = x[2]
        roll = x[3]
        pitch = x[4]
        yaw = x[5]

        xVel = x[6]
        yVel = x[7]
        zVel = x[8]
        rollVel = x[9]
        pitchVel = x[10]
        yawVel = x[11]

        v1 = x[6:9]
        v2 = x[9:12]


        cgX = self.centerOfGravity[0]
        cgY = self.centerOfGravity[1]
        cgZ = self.centerOfGravity[2]

        cbX = self.centerOfBuoyancy[0]
        cbY = self.centerOfBuoyancy[1]
        cbZ = self.centerOfBuoyancy[2]

        # Coriolis and centripital forces
        # Fossen 2011, p. 65
        C11 = np.identity(3)
        C12 = -self.mass * self.S(v1) - self.mass*np.matmul(self.S(v2), self.S(self.centerOfGravity))
        C21 = -1*np.transpose(C12)
        C22 = -self.S(np.matmul(self.inertiaTensor, v2))

        CRB  = np.vstack((np.hstack((C11, C12)), np.hstack((C21, C22))))

        MaX = self.addedMass[0,0]
        MaY = self.addedMass[1,1]
        MaZ = self.addedMass[2,2]
        MaRoll = self.addedMass[3,3]
        MaPitch = self.addedMass[4,4]
        MaYaw = self.addedMass[5,5]


        # Fossen 2011, p. 121
        Ca = np.array([
                            [0, 0, 0, 0, -MaZ*zVel, MaY*yVel],
                            [0, 0, 0, MaZ*zVel, 0, -MaX*xVel],
                            [0, 0, 0, -MaY*yVel, MaX*xVel, 0],
                            [0, -MaZ*zVel, MaY*yVel, 0, -MaYaw*yawVel, MaPitch*pitchVel],
                            [MaZ*zVel, 0, -MaX*xVel, MaYaw*yawVel, 0, -MaRoll*rollVel],
                            [-MaY*yVel, MaX*xVel, 0, -MaPitch*pitchVel, MaRoll * rollVel, 0]
                          ])

        C = Ca + CRB


        # Masses
        Mm = self.mass * np.identity(3)

        np.hstack((self.mass*self.S(self.centerOfGravity), self.inertiaTensor))

        MRB = np.vstack((np.hstack((Mm, -self.mass*self.S(self.centerOfGravity))),
                        np.hstack((self.mass*self.S(self.centerOfGravity), self.inertiaTensor))))
        M = self.addedMass + MRB



        # Transformation between world and robot frame
        # OK sorry... renamed for the sake of consistency with textbook
        phi = roll
        theta = pitch
        tsi = yaw
        cPhi = np.cos(phi);
        sPhi = np.sin(phi);
        cTheta = np.cos(theta);
        sTheta = np.sin(theta);
        tTheta = np.tan(theta);
        cTsi = np.cos(tsi);
        sTsi = np.sin(tsi);

        # Fossen 2011, p. 22
        Rb = np.array([[cTsi*cTheta, (-sTsi*cPhi + cTsi*sTheta*sPhi), (sTsi*sPhi+cTsi*cPhi*sTheta)],
                        [sTsi*cTheta, (cTsi*cPhi + sPhi*sTheta*sTsi), (-cTsi*sPhi+sTheta*sTsi*cTheta)],
                        [-sTheta, cTheta*sPhi, cTheta*cPhi]])

        # Fossen 2011, p. 25
        Tt = np.array([[1, sPhi*tTheta, cPhi*tTheta],
                       [0, cPhi, -sPhi],
                       [0, (sPhi/cTheta), (cPhi/cTheta)]]);

        # Fossen 2011, p. 26
        Jn = np.vstack((np.hstack((Rb, np.zeros((3,3)))), np.hstack((np.zeros((3,3)),Tt))))

        # Put force of gravity in frame of robot
        # Fossen 2011, p. 60
        self.Gf = np.array([[self.effectiveWeight* sTheta],
                      [-self.effectiveWeight * cTheta*sPhi],
                      [-self.effectiveWeight * cTheta*cPhi],
                      [-(cgY*self.weight - cbY*self.buoyancy)*cTheta*cPhi+(cgZ*self.weight-cbZ*self.buoyancy)*cTheta*sPhi],
                      [(cgZ*self.weight-cbZ*self.buoyancy)*sTheta+(cgX*self.weight-cbX*self.buoyancy)*cTheta*cPhi],
                      [-(cgX*self.weight-cbX*self.buoyancy)*cTheta*sPhi-(cgY*self.weight-cbY*self.buoyancy)*sTheta]])


        mInv = np.zeros((6,6))
        if np.linalg.det(M) != 0:
            mInv = np.linalg.inv(M)

        #Chin 2013, p. 138
        f1 = np.vstack((np.hstack((np.zeros((6,6)), Jn)), np.hstack((np.zeros((6,6)), -np.matmul(mInv,(C+self.D))))))
        f2 = np.vstack((np.zeros((6,1)), -np.matmul(mInv,self.Gf)))
        f = np.matmul(f1,x.reshape((12,1))) + f2
        g = np.vstack((np.zeros((6,1)), np.matmul(mInv,u).reshape((6,1))))
        return f+g


    def S(self, p):

        return np.array([[0, -p[2], p[1]],
                        [p[2], 0, -p[0]],
                        [-p[1], p[0], 0]])

    def simLQR(self, duration):
        dt = 1/self.rate

        for i in range(int(self.rate*duration)):
            linearizedSystem = control.linearize(self.nonlinearIoSys, self.x, self.u)
            K,_,_ = control.lqr(linearizedSystem, self.Q, self.R)

            err = self.x - self.xRef

            for i in range(3,6):
                err[i] = min(abs(err[i]), abs(360-err[i]))*np.sign(err[i])

            self.u = np.matmul(K, err.reshape((12,1)))

            xDot = self(0, self.x, self.u)

            self.x += xDot * dt


if __name__ == '__main__':
    rospy.init_node("VehicleDynamics")
    dynamics = VehicleDynamics()

    dt = 1/dynamics.rate

    dynamics.xRef = np.hstack((np.ones(3), np.zeros(9)))

    sys = control.iosys.NonlinearIOSystem(dynamics, inputs=6, outputs=12, states=12, name="RobotDynamics")

    dynamics.simLQR(0.1)
