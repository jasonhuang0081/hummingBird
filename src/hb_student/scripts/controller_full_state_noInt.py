#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate
import numpy as np
import control as ct


class Controller_fullState(ControllerBase):
    def init_control(self, param):
        self.wn_phi = 2 #3   #rotation
        self.wn_psi = 10 #10     # side way
        self.wn_theta = 5    # height
        self.z_phi = 0.707
        self.z_psi = 0.707
        self.z_theta = 0.707
        self.integrator_psi = -5
        self.integrator_theta = -10
        self.param = param
        self.theta_prev = 0
        self.thetadot_prev = 0
        self.phi_prev = 0
        self.phidot_prev = 0
        self.psi_prev = 0
        self.psidot_prev = 0
        self.sigma = 0.005
        self.error_inte_pitch = 0
        self.error_inte_yaw = 0
        self.error_pitch_prev = 0
        self.error_yaw_prev = 0
        self.error_pitch_dot = 0
        self.error_yaw_dot = 0
        self.cal_feedback(param)

        self.km = 1.029  # for simulation
        # self.km = 0.276

    def derivative(self, current, dt, prev, pre_dot):
        return (2*self.sigma - dt)/(2*self.sigma + dt)*pre_dot + 2/(2*self.sigma + dt)*(current - prev)

    def cal_feedback(self, param):
        theta_0 = 0
        b_theta = param.lT/(param.m1*param.l1**2 + param.m2*param.l2**2 + param.J1y + param.J2y)
        A_lon = np.zeros((2,2))
        A_lon[0,1] = 1
        B_lon = np.zeros((2,1))
        B_lon[1,0] = b_theta
        C_lon = np.zeros((1,2))
        C_lon[0,0] = 1

        self.Feq = (self.param.m1*self.param.l1 + self.param.m2*self.param.l2)*self.param.g*np.cos(theta_0)/self.param.lT
        JT = self.param.m1*self.param.l1**2 + self.param.m2*self.param.l2**2 + self.param.J2z + self.param.m3*(self.param.l3x**2 + self.param.l3y**2)

        A_lat = np.zeros((4,4))
        A_lat[0,2] = 1
        A_lat[1,3] = 1
        A_lat[3,0] = self.param.lT*self.Feq/(JT+self.param.J1z)
        B_lat = np.zeros((4,1))
        B_lat[2,0] = 1/(self.param.J1x)
        C_lat = np.zeros((2,4))
        C_lat[0,0] = 1
        C_lat[1,1] = 1
        Cr_yaw = np.zeros((1,4))
        Cr_yaw[0,1] = 1

        des_char_lon = np.array([1,2*self.z_theta*self.wn_theta,self.wn_theta**2])
        #print(ct.ctrb(A_lon,B_lon))
        des_pole_lon = np.roots(des_char_lon)
        self.K_lon = ct.place(A_lon,B_lon,des_pole_lon)
        self.kr_lon = -1/(np.dot(np.dot(C_lon, np.linalg.inv(A_lon-B_lon*self.K_lon)),B_lon))

        print(self.K_lon)
        print(self.kr_lon)


        des_char_lat = np.convolve([1,2*self.z_phi*self.wn_phi,self.wn_phi**2],[1,2*self.z_psi*self.wn_psi,self.wn_psi**2])
        des_pole_lat = np.roots(des_char_lat)
        #(ct.ctrb(A_lat,B_lat))
        self.K_lat = ct.place(A_lat,B_lat,des_pole_lat)
        self.kr_lat = -1/(np.dot(np.dot(Cr_yaw, np.linalg.inv(A_lat-B_lat*self.K_lat)), B_lat))

        print(self.K_lat)
        print(self.kr_lat)

    def compute_control(self, param, state, reference, dt):
        left = 0.0
        right = 0.0

        theta = state.pitch
        phi = state.roll
        psi = state.yaw
        r_theta = reference.pitch
        r_psi = reference.yaw

        error_pitch = r_theta - theta
        error_yaw = r_psi  - psi

        self.error_pitch_dot = self.derivative(error_pitch, dt, self.error_pitch_prev, self.error_pitch_dot)
        self.error_yaw_dot = self.derivative(error_yaw, dt, self.error_yaw_prev, self.error_yaw_dot)

        thetadot = self.derivative(theta, dt, self.theta_prev, self.thetadot_prev)
        phidot = self.derivative(phi,dt, self.phi_prev, self.phidot_prev)
        psidot = self.derivative(psi,dt, self.psi_prev, self.psidot_prev)
        self.theta_prev = theta
        self.thetadot_prev = thetadot
        self.phidot_prev = phidot
        self.phi_prev = phi
        self.psidot_prev = psidot
        self.psi_prev = psi

        x_lon = np.array([[theta],[thetadot]])
        x_lat = np.array([[phi],[psi],[phidot],[psidot]])

        F_np = self.Feq - np.dot(self.K_lon, x_lon) + self.kr_lon*(r_theta)
        #print(F_np)
        F = F_np[0,0]

        tau_np = -1*np.dot(self.K_lat, x_lat) + self.kr_lat*(r_psi)

        print(tau_np)
        tau = tau_np[0,0]


        left = 1/(2*self.km)*(F + tau/param.d)
        right = 1/(2*self.km)*(F - tau/param.d)

        left = saturate(left, 0, 0.7)
        right = saturate(right, 0, 0.7)

        return Command(left=left, right=right)


if __name__ == '__main__':
    c = Controller_fullState()
    c.run()
