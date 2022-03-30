"""Module to abstract the functionalities of the GPR-20 Cartesian arm."""

import rospy
import actionlib
from std_srvs.srv import Empty
from gpr20_msgs.msg import GPR20AxisStatus
from gpr20_msgs.msg import AxisAction, AxisGoal


class CartesianArm(object):
    """Class to handle the Cartesian arm for the GPR-20 control.

    Attributes:
        x_axis_client (actionlib.SimpleActionClient): client for the axis
            action server. This allows to command the X-Axis of the robot.
        y_axis_client (actionlib.SimpleActionClient): client for the axis
            action server. This allows to command the Y-Axis of the robot.
        z_axis_client (actionlib.SimpleActionClient): client for the axis
            action server. This allows to command the Z-Axis of the robot.
        x_goal (gpr20_msgs.msg.AxisGoal): goal for the X-Axis action.
        y_goal (gpr20_msgs.msg.AxisGoal): goal for the Y-Axis action.
        z_goal (gpr20_msgs.msg.AxisGoal): goal for the Z-Axis action.
        x_axis_homing_srv (rospy.ServiceProxy): service proxy to execute the
            homing sequence on the X-Axis.
        y_axis_homing_srv (rospy.ServiceProxy): service proxy to execute the
            homing sequence on the Y-Axis.
        z_axis_homing_srv (rospy.ServiceProxy): service proxy to execute the
            homing sequence on the Z-Axis.
    """
    # Set the X-Axis action name
    X_AXIS_ACTION_NAME = "/gpr20_x_axis/axis"

    # Set the Y-Axis action name
    Y_AXIS_ACTION_NAME = "/gpr20_y_axis/axis"

    # Set the Z-Axis action name
    Z_AXIS_ACTION_NAME = "/gpr20_z_axis/axis"

    def __init__(self):
        """Initialize the class attributes for the Cartesian arm handler.

        The initialization consists of creating the action servers for the
        three (3) axes of the robot. The initialization checks that the action
        servers are available and then proceeds to subscribe to the current
        coordinates topics of the robot. Additional attributes are created to
        be reutilized during the robot operation.
        """
        # Create the client for the X-axis
        self.x_axis_client = actionlib.SimpleActionClient(
            self.X_AXIS_ACTION_NAME,
            AxisAction
        )

        # Wait for X-axis server to be available
        self.x_axis_client.wait_for_server()

        # Create the client for the Y-axis
        self.y_axis_client = actionlib.SimpleActionClient(
            self.Y_AXIS_ACTION_NAME,
            AxisAction
        )

        # Wait for Y-axis server to be available
        self.y_axis_client.wait_for_server()

        # Create the client for the Z-axis
        self.z_axis_client = actionlib.SimpleActionClient(
            self.Z_AXIS_ACTION_NAME,
            AxisAction
        )

        # Wait for Z-axis server to be available
        self.z_axis_client.wait_for_server()

        # Create the goal for X-axis
        self.x_goal = AxisGoal()

        # Create the goal for Y-axis
        self.y_goal = AxisGoal()

        # Create the goal for Z-axis
        self.z_goal = AxisGoal()

        # Create the proxy for the X-axis homing
        self.x_axis_homing_srv = rospy.ServiceProxy(
            "gpr20_x_axis/homing",
            Empty
        )

        # Create the proxy for the Y-axis homing
        self.x_axis_homing_srv = rospy.ServiceProxy(
            "gpr20_x_axis/homing",
            Empty
        )

        # Create the proxy for the Z-axis homing
        self.x_axis_homing_srv = rospy.ServiceProxy(
            "gpr20_x_axis/homing",
            Empty
        )

        # Set the homing variables
        self.x_axis_homing, self.y_axis_homing = False, False

        # Create the subscriber for the X-Axis status
        rospy.Subscriber(
            "gpr20_x_axis/axis_status",
            GPR20AxisStatus,
            self.x_axis_status_cb
        )

        # Create the subscriber for the Y-Axis status
        rospy.Subscriber(
            "gpr20_y_axis/axis_status",
            GPR20AxisStatus,
            self.y_axis_status_cb
        )

    def x_axis_homing(self):
        """Call the homing service for the X-Axis."""
        self.x_axis_homing_srv()

    def y_axis_homing(self):
        """Call the homing service for the Y-Axis."""
        self.y_axis_homing_srv()

    def z_axis_homing(self):
        """Call the homing service for the Z-Axis."""
        self.z_axis_homing_srv()

    def get_homing_status(self):
        """Get the global homing status for Cartesian arm.

        Returns:
            bool: flag indicating if the three axes executed the homing
                sequence.
        """
        return self.x_axis_homing and self.y_axis_homing

    def x_axis_status_cb(self, msg):
        """Callback for the X-Axis status topic.

        Args:
            msg (gpr20_msg.msg.GPR20AxisStatus): status message sent by the
                axis driver.
        """
        # Store the homing status in attribute
        self.x_axis_homing = msg.homing

    def y_axis_status_cb(self, msg):
        """Callback for the Y-Axis status topic.

        Args:
            msg (gpr20_msg.msg.GPR20AxisStatus): status message sent by the
                axis driver.
        """
        # Store the homing status in attribute
        self.y_axis_homing = msg.homing

    def go_to_coordinates(self, x_coord, y_coord, z_coord):
        """Moves cartesian arm to a given position.

        Args:
            x_coord (float): target coordinate for X-Axis.
            y_coord (float): target coordinate for Y-Axis.
            z_coord (float): target coordinate for Z-Axis.
        """
        # Update the goals with target coordinates
        self.x_goal.target_coordinate = x_coord
        self.y_goal.target_coordinate = y_coord
        self.z_goal.target_coordinate = z_coord

        # Sends goals
        self.x_axis_client.send_goal(self.x_goal)
        self.y_axis_client.send_goal(self.y_goal)
        self.z_axis_client.send_goal(self.y_goal)

        # Waits until reaching target to return
        self.x_axis_client.wait_for_result()
        self.y_axis_client.wait_for_result()
        self.z_axis_client.wait_for_result()
