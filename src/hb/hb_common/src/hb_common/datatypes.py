import rospy
from typing import NamedTuple


class ROSParameterException(Exception):
    """Exception indicating an error occurred retrieving values for the ROS parameter server
    """
    pass


class Params(NamedTuple):
    l1: float
    l2: float
    l3x: float
    l3y: float
    l3z: float
    lT: float
    d: float
    m1: float
    m2: float
    m3: float
    Bphi: float
    Bth: float
    Bpsi: float
    J1x: float
    J1y: float
    J1z: float
    J2x: float
    J2y: float
    J2z: float
    J3x: float
    J3y: float
    J3z: float
    km: float
    g: float
    sigma_gyro: float
    sigma_pixel: float
    phi_min: float
    phi_max: float
    theta_min: float
    theta_max: float
    psi_min: float
    psi_max: float
    disturbances: bool
    disturbance_percent: float
    dynamics_rate: float

    @staticmethod
    def _from_rosparam(namespace='/hummingbird'):
        try:
            param = rospy.get_param(namespace)
        except KeyError:
            raise ROSParameterException(
                "Parameters not set in the {} namespace".format(namespace))

        try:
            obj = Params(l1=param['l1'],
                         l2=param['l2'],
                         l3x=param['l3x'],
                         l3y=param['l3y'],
                         l3z=param['l3z'],
                         lT=param['lT'],
                         d=param['d'],
                         m1=param['m1'],
                         m2=param['m2'],
                         m3=param['m3'],
                         Bphi=param['Bphi'],
                         Bth=param['Bth'],
                         Bpsi=param['Bpsi'],
                         J1x=param['J1x'],
                         J1y=param['J1y'],
                         J1z=param['J1z'],
                         J2x=param['J2x'],
                         J2y=param['J2y'],
                         J2z=param['J2z'],
                         J3x=param['J3x'],
                         J3y=param['J3y'],
                         J3z=param['J3z'],
                         km=param['km'],
                         g=param['g'],
                         sigma_gyro=param['sigma_gyro'],
                         sigma_pixel=param['sigma_pixel'],
                         phi_min=param['phi_min'],
                         phi_max=param['phi_max'],
                         theta_min=param['theta_min'],
                         theta_max=param['theta_max'],
                         psi_min=param['psi_min'],
                         psi_max=param['psi_max'],
                         disturbances=param['disturbances'],
                         disturbance_percent=param['disturbance_percent'],
                         dynamics_rate=param['dynamics_rate'])
            # obj = Params(**param)
        except Exception as e:
            raise ROSParameterException(
                "Did not receive expected parameters: {}".format(e))

        return obj


class Command(NamedTuple):
    left: float
    right: float


class State(NamedTuple):
    roll: float
    pitch: float
    yaw: float


class ReferenceState(NamedTuple):
    pitch: float
    yaw: float
