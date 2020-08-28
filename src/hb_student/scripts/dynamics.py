#!/usr/bin/env python3

import numpy as np

from hb_common import DynamicsBase, Params, Command


class Dynamics(DynamicsBase):
    def __init__(self):
        super().__init__()

    def dynamics(self, param, state, command):
        l1 = param.l1
        l2 = param.l2
        l3x = param.l3x
        l3y = param.l3y
        l3z = param.l3z
        lT = param.lT
        d = param.d
        m1 = param.m1
        m2 = param.m2
        m3 = param.m3
        J1x = param.J1x
        J1y = param.J1y
        J1z = param.J1z
        J2x = param.J2x
        J2y = param.J2y
        J2z = param.J2z
        J3x = param.J3x
        J3y = param.J3y
        J3z = param.J3z
        Bphi = param.Bphi
        Bth = param.Bth
        Bpsi = param.Bpsi
        km = param.km
        g = param.g

        phi = state[0]
        theta = state[1]
        psi = state[2]
        phid = state[3]
        thetad = state[4]
        psid = state[5]

        # forces
        fl = km * command.left
        fr = km * command.right

        # angle rate dynamics
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi = np.sin(psi)
        cpsi = np.cos(psi)

        ################################################
        # Implement Dynamics for Accelerations Here    #
        N33 = 2*(J1x + J2x - m1*l1**2 - m2*l2**2 - J2z - J1y*sphi**2 - J1z*cphi**2)*stheta*ctheta

        M = np.zeros((3, 3))
        M[0, 0] = J1x
        M[0, 2] = -1*J1x*stheta
        M[1, 1] = m1*l1**2 + m2*l2**2 + J2y + J1y*cphi**2 + J1z*sphi**2
        M[1, 2] = (J1y - J1z)*sphi*cphi*ctheta
        M[2, 0] = -1*J1x*stheta
        M[2, 1] = (J1y - J1z)*sphi*cphi*ctheta
        M[2, 2] = (m1*l1**2 + m2*l2**2 + J2z + J1y*sphi**2 + J1z*cphi**2)*ctheta**2 + (J1x + J2x)*stheta**2 + m3*(l3x**2 + l3y**2) + J3z

        c = np.zeros((3, 1))
        c[0] =(J1y - J1z)*sphi*cphi*(thetad**2 - ctheta**2*psid**2) + ((J1y - J1z)*(cphi**2 - sphi**2) - J1x)*ctheta*thetad*psid
        c[1] = 2*(J1z - J1y)*sphi*cphi*phid*thetad + ((J1y - J1z)*(cphi**2 - sphi**2) + J1x)*ctheta*phid*psid - 1/2*N33*psid**2
        c[2] = thetad**2*(J1z - J1y)*sphi*cphi*stheta + ((J1y - J1z)*(cphi**2 - sphi**2) - J1z)*ctheta*phid*thetad + (J1z - J1y)*sphi*cphi*stheta*thetad**2 + 2*(J1y - J1z)*sphi*cphi*phid*psid + 2*(-1*m1*l1**2 - m2*l2*82 -J2z + J1x + J2x + J1y*sphi**2 + J1z*sphi**2)*stheta*ctheta*thetad*psid

        print(c)
        dPdq = np.zeros((3, 1))
        dPdq[1] = (m1*l1 + m2*l2)*g*ctheta

        Tau = np.zeros((3, 1))
        Tau[0] = d*(fl - fr)
        Tau[1] = lT*(fl + fr)*cphi
        Tau[2] = lT*(fl + fr)*ctheta*sphi - d*(fl - fr)*stheta

        B = np.zeros((3, 3))
        B[0, 0] = Bphi
        B[1, 1] = Bth
        B[2, 2] = Bpsi

        ################################################

        xdot = np.zeros((6, 1))
        xdot[0:3] = state[3:6]  # angle dynamics
        xdot[3:6] = np.linalg.solve(
            M, Tau - B@xdot[0:3] - c - dPdq)  # angle rate dynamics

        return xdot


if __name__ == '__main__':
    d = Dynamics()
    d.run()
