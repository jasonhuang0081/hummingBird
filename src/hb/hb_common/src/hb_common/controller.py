import rospy
import std_msgs.msg
import hb_msgs.msg

import numpy as np

from hb_common.datatypes import Params, Command, State, ReferenceState, ROSParameterException
from hb_common.helpers import saturate


class ControllerBase:
    """Base class for hummingbird controllers.

    Public Methods:
        run() -- Runs the controller node
        armed() -- Returns the armed status of the hummingbird

    This class must be subclassed. The subclass must implement the following
    methods:
        init_control(param) -- Perform any controller initialization actions
        compute_control(param, state, reference, dt) -- Run the controller and return motor commands
    """

    def __init__(self):
        pass

    def _setup(self):
        try:
            self._param = Params._from_rosparam()
        except ROSParameterException as e:
            rospy.logfatal("Error getting parameters: {}".format(e))
            rospy.signal_shutdown("Error getting parameters")
            return

        self.init_control(self._param)

        self._prev_time = None
        self._reference_state = ReferenceState(pitch=0.0, yaw=0.0)
        self._armed = False

        self._state_sub = rospy.Subscriber(
            "hb_state", hb_msgs.msg.State, self._state_cb, queue_size=1)
        self._reference_sub = rospy.Subscriber(
            "hb_reference_state", hb_msgs.msg.ReferenceState, self._reference_cb, queue_size=1)
        self._arm_status_sub = rospy.Subscriber(
            "hb_arm_status", std_msgs.msg.Bool, self._arm_status_cb, queue_size=5)
        self._command_pub = rospy.Publisher(
            "hb_command", hb_msgs.msg.Command, queue_size=1)

    def _reference_cb(self, msg):
        self._reference_state = ReferenceState(pitch=msg.pitch, yaw=msg.yaw)

    def _arm_status_cb(self, msg):
        self._armed = msg.data

    def _state_cb(self, msg):
        if self._prev_time is None:
            self._prev_time = rospy.Time.now()
            return

        time = rospy.Time.now()
        dt = (time - self._prev_time).to_sec()
        self._prev_time = time

        state = State(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
        command = self.compute_control(
            self._param, state, self._reference_state, dt)

        cmd_msg = hb_msgs.msg.Command(
            left_motor=command.left, right_motor=command.right)
        self._command_pub.publish(cmd_msg)

    def run(self):
        """Runs the controller node."""
        rospy.init_node("controller")
        self._setup()
        rospy.spin()

    def armed(self):
        """Returns the armed status of the hummingbird.

        This can be useful for deciding whether to update integrators.

        Returns:
            bool -- True if the hummingbird is armed
        """
        return self._armed

    def init_control(self, param):
        """Perform any initialization steps for the controller.

        This function should perform initialization steps such as pre-computing
        controller gains.

        Arguments:
            param {Params} -- The system parameters, accessed as param.m1, etc.

        Raises:
            NotImplementedError: This function must be overridden by the subclass
        """
        raise NotImplementedError()

    def compute_control(self, param, state, reference, dt):
        """Computes the control command.

        Arguments:
            param {hb_common.Params} -- The system parameters; accessed as param.m1, etc.
            state {hb_common.State} -- The encoder angles; accessed as state.yaw, state.pitch, state.roll
            reference {hb_common.ReferenceState} -- The reference commands; accessed as reference.yaw and reference.pitch
            dt {float} -- The time since the last call to this function, in seconds

        Returns:
            {hb_common.Command} -- The motor throttle commands (left and right)

        Raises:
            NotImplementedError: This function must be overridden by the subclass
        """
        raise NotImplementedError()
