#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate
import numpy as np


class Controller_4(ControllerBase):
    def init_control(self, param):
        tr = 1.4
        zeta = 0.707
        tr_r = 0.2
        zeta_r = 0.7
        tr_y = tr_r*10
        zeta_y = 0.7
        reaction_factor = 2
        self.ki_pitch = 0.05
        self.ki_yaw = 0.05

        wn_r = 2.2/tr_r
        wn_y = 2.2/tr_y
        DCgain = 1
        Fe = (param.m1*param.l1 + param.m2*param.l2)*param.g*np.cos(0)/param.lT
        JT = param.m1*param.l1**2 + param.m2*param.l2**2 + param.J2z + param.m3*(param.l3x**2 + param.l3y**2)
        b_psi = param.lT*Fe/(param.J1z + JT)
        self.kd_r = 2*zeta_r*wn_r*param.J1x
        self.kp_r = (param.J1x*wn_r**2)
        self.kd_y = (2*zeta_y*wn_y/b_psi)     #3
        self.kp_y = ((wn_y**2/b_psi)*reaction_factor)
        b_theta = param.lT/(param.m1*param.l1**2 + param.m2*param.l2**2 + param.J1y + param.J2y)
        self.kd_p = 4.4*zeta/(tr*b_theta)  #5
        self.kp_p = 4.84/(b_theta*tr**2)  #2
        self.sigma = 0.005
        self.theta_prev = 0
        self.thetadot_prev = 0
        self.phi_prev = 0
        self.phidot_prev = 0
        self.psi_prev = 0
        self.psidot_prev = 0
        self.error_inte_pitch = 0
        self.error_inte_yaw = 0
        self.error_pitch_prev = 0
        self.error_yaw_prev = 0
        self.error_pitch_dot = 0
        self.error_yaw_dot = 0
        #self.km = 1.029  # for simulation
        self.km = 0.276

    def derivative(self, current, dt, prev, pre_dot):
        return (2*self.sigma - dt)/(2*self.sigma + dt)*pre_dot + 2/(2*self.sigma + dt)*(current - prev)

    def integrate(self, dt, z, z_prev, z_inte_prev):
        return z_inte_prev + dt/2*(z+z_prev)

    def integratorAntiWindup(self, u_sat, u_unsat, dt):
        if self.ki_yaw != 0:
            self.error_inte_yaw = self.error_inte_yaw + dt/self.ki_yaw*(u_sat - u_unsat)
        if self.ki_pitch != 0:
            self.error_inte_pitch = self.error_inte_pitch + dt/self.ki_pitch*(u_sat - u_unsat)

    def compute_control(self, param, state, reference, dt):
        left = 0.0
        right = 0.0
        theta = state.pitch
        phi = state.roll
        psi = state.yaw

        error_pitch = reference.pitch - theta
        error_yaw = reference.yaw - psi

        self.error_pitch_dot = self.derivative(error_pitch, dt, self.error_pitch_prev, self.error_pitch_dot)
        self.error_yaw_dot = self.derivative(error_yaw, dt, self.error_yaw_prev, self.error_yaw_dot)
        if np.abs(self.error_pitch_dot) < 1:
            self.error_inte_pitch = self.integrate(dt, error_pitch,self.error_pitch_prev,self.error_inte_pitch)
            #print(self.error_pitch_dot)
        self.error_pitch_prev = error_pitch
        if np.abs(self.error_yaw_dot) < 1:
            self.error_inte_yaw = self.integrate(dt, error_yaw, self.error_yaw_prev, self.error_inte_yaw)
            #print(self.error_yaw_dot)
        self.error_yaw_prev = error_yaw

        thetadot = self.derivative(theta, dt, self.theta_prev, self.thetadot_prev)
        phidot = self.derivative(phi,dt, self.phi_prev, self.phidot_prev)
        psidot = self.derivative(psi,dt, self.psi_prev, self.psidot_prev)

        F_tilde = self.kp_p*(reference.pitch - theta) - self.kd_p*thetadot + self.ki_pitch*self.error_inte_pitch
        roll_r = self.kp_y*(reference.yaw - psi) - self.kd_y*psidot + self.ki_yaw*self.error_inte_yaw
        tau = self.kp_r*(roll_r - phi) - self.kd_r*phidot
        self.theta_prev = theta
        self.thetadot_prev = thetadot
        self.phidot_prev = phidot
        self.phi_prev = phi
        self.psidot_prev = psidot
        self.psi_prev = psi

        Feq = (param.m1*param.l1 + param.m2*param.l2)*param.g*np.cos(theta)/param.lT
        F = Feq + F_tilde
        #F_sat = saturate(F,0,1.0)
        #self.integratorAntiWindup(F_sat, F, dt)


        left = 1/(2*self.km)*(F + tau/param.d)
        right = 1/(2*self.km)*(F - tau/param.d)

        left = saturate(left, 0, 0.7)
        right = saturate(right, 0, 0.7)

        return Command(left=left, right=right)


if __name__ == '__main__':
    c = Controller_4()
    c.run()
