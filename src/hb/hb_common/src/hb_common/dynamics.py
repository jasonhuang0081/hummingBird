import rospy
import std_msgs.msg
import hb_msgs.msg

import numpy as np

from hb_common.datatypes import Params, Command, ROSParameterException
from hb_common.helpers import saturate


class DynamicsBase:
    """Base class for the hummingbird dynamics simulator.

    Public Methods:
        run() -- Runs the simulator node

    This class must be subclassed. The subclass must implement the following
    methods:
        dynamics(param, state, command) -- Compute the hummingbird state derivatives
    """

    PHI = 0
    THETA = 1
    PSI = 2
    PHID = 3
    THETAD = 4
    PSID = 5

    def __init__(self):
        pass

    def _setup(self):
        try:
            self._param = Params._from_rosparam()
        except ROSParameterException as e:
            rospy.logfatal("Error getting parameters: {}".format(e))
            rospy.signal_shutdown("Error getting parameters")
            return

        if self._param.disturbances:  # apply mass disturbance
            self._param = self._param._replace(
                m1=(np.random.random()*2 - 1)*self._param.m1*self._param.disturbance_percent + self._param.m1)

        self._state = np.zeros((6, 1))
        self._command = Command(left=0.0, right=0.0)
        self._armed = False
        self._initialized = False

        self._command_sub = rospy.Subscriber(
            '/hb_command', hb_msgs.msg.Command, self._command_callback)
        self._state_pub = rospy.Publisher(
            '/hb_state', hb_msgs.msg.State, queue_size=1)
        self._arm_status_pub = rospy.Publisher(
            '/hb_arm_status', std_msgs.msg.Bool, latch=True, queue_size=1)

        self._arm_status_pub.publish(False)

        self._dynamics_timer = rospy.Timer(
            rospy.Duration(1.0/self._param.dynamics_rate), self._timer_callback)

    def _command_callback(self, msg):
        if not self._armed:
            self._armed = True
            self._arm_status_pub.publish(True)

        if self._param.disturbances:  # apply ESC disturbance
            disturbance = (np.random.random()*2 - 1) * \
                self._param.disturbance_percent * 0.1
        else:
            disturbance = 0.0

        self._command = Command(
            left=saturate(msg.left_motor + disturbance, 0.0, 1.0),
            right=saturate(msg.right_motor, 0.0, 1.0))

    def _timer_callback(self, event):
        if not self._initialized:
            self._initialized = True
            return

        self._propagate((event.current_real - event.last_real).to_sec())

        roll, pitch, yaw = self._encoder_values()
        self._state_pub.publish(roll=roll, pitch=pitch, yaw=yaw)

    def _propagate(self, dt):
        # RK4 integration
        k1 = self.dynamics(self._param, self._state, self._command)
        k2 = self.dynamics(self._param, self._state + dt/2*k1, self._command)
        k3 = self.dynamics(self._param, self._state + dt/2*k2, self._command)
        k4 = self.dynamics(self._param, self._state + dt*k3, self._command)
        self._state += dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        # Implement hard stops on angles
        def hard_stop(angle_idx, rate_idx, angle_min, angle_max):
            if self._state[angle_idx] > angle_max:
                self._state[angle_idx] = angle_max
                if self._state[rate_idx] > 0:
                    self._state[rate_idx] *= -0.5  # bounce against limit

            if self._state[angle_idx] < angle_min:
                self._state[angle_idx] = angle_min
                if self._state[rate_idx] < 0:
                    self._state[rate_idx] *= -0.5  # bounce against limit

        hard_stop(self.PHI,   self.PHID,
                  self._param.phi_min,   self._param.phi_max)
        hard_stop(self.THETA, self.THETAD,
                  self._param.theta_min, self._param.theta_max)
        hard_stop(self.PSI,   self.PSID,
                  self._param.psi_min,   self._param.psi_max)

    def _encoder_values(self):
        return self._state[self.PHI], self._state[self.THETA], self._state[self.PSI]

    def run(self):
        """Runs the dynamics simulation node."""
        rospy.init_node("dynamics")
        self._setup()
        rospy.spin()

    def dynamics(self, param, state, command):
        """Compute the dynamics model.

        Arguments:
            param {hb_common.Params} -- The system parameters; accessed as param.m1, etc.
            state {numpy.ndarray} -- The current system state [phi,theta,psi,phid,thetad,psid]
            command {hb_common.Command} -- The current motor throttle commands; accessed as command.left, command.right

        Returns:
            {numpy.ndarray} -- The system dynamics derivative [phid,thetad,psid,phidd,thetadd,psidd]

        Raises:
            NotImplementedError: This function must be overridden by the subclass
        """
        raise NotImplementedError()
