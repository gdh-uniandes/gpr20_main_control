"""Module to abstract the data acquisition calls to GPR-20 utilities."""

import rospy
from gpr20_msgs.srv import GetHeight
from gpr20_msgs.srv import VNAGetData
from gpr20_msgs.srv import VNAGetFreq
from gpr20_msgs.srv import VNAConnection
from gpr20_msgs.srv import VNASweepSetup
from gpr20_msgs.srv import VNACalibrationStatus


class DataAcquisition(object):
    """Class to provide calling conventions to data acquisition utilities.

    DataAcquisition instantiates the calling objects used by ROS in order to
    ease the access to data acquistion utilities. The calling objects are
    instantiated as class attributes in order to be continously used by the
    calling methods.

    Attributes:
        vna_conn_srv (rospy.ServiceProxy): calling object to configure the VNA
            instrument connection in the 'GPR-20 VNA Acquisition' utility.
        vna_cal_status_srv (rospy.ServiceProxy): calling object to retrieve
            the VNA calibration status from the 'GPR-20 VNA Acquisition'
            utility.
        vna_config_freq_sweep _srv (rospy.ServiceProxy): calling object to
            configure the frequency sweep in the VNA instrument as provided
            by the 'GPR-20 VNA Acquisition' utility.
        vna_get_freq_srv (rospy.ServiceProxy): calling object to retrieve the
            VNA frequencies vector as provided by the 'GPR-20 VNA Acquisiton'
            utility.
        vna_get_data_srv (rospy.ServiceProxy): calling object to retrieve the
            trace VNA data as provided by the 'GPR-20 VNA Acquisition'.
        antennae_height_srv (rospy.ServiceProxy): calling object to retrieve
            the height value as provided by the 'GPR-20 Height' utility.
    """

    def __init__(self):
        """Initialize the class that handles the data acquisiton.

        This class creates instances for calling the services related to the
        robot data acquisition activities. The data acquisition is linked to
        the VNA data and the antennae height data.
        """
        # Create the service proxy for VNA connection
        self.vna_conn_srv = rospy.ServiceProxy(
            "vna_connection",
            VNAConnection
        )

        # Create the service proxy for checking VNA calibration status
        self.vna_cal_status_srv = rospy.ServiceProxy(
            "vna_get_calibration_status",
            VNACalibrationStatus
        )

        # Create the service proxy for configuring the VNA freq. sweep
        self.vna_config_freq_sweep_srv = rospy.ServiceProxy(
            "vna_freq_sweep_setup",
            VNASweepSetup
        )

        # Create the service proxy for acquiring the VNA freq. data
        self.vna_get_freq_srv = rospy.ServiceProxy(
            "vna_get_freq",
            VNAGetFreq
        )

        # Create the service proxy for acquiring the VNA trace data
        self.vna_get_data_srv = rospy.ServiceProxy(
            "vna_get_data",
            VNAGetData
        )

        # Create the service proxy for acquiring the antennae height
        self.antennae_height_srv = rospy.ServiceProxy(
            "get_height",
            GetHeight
        )

    def connect_to_vna(self, ip_addr):
        """Command the VNA driver to connect to instrument.

        Args:
            ip_addr (str): IP address that the VNA driver will attempt to
                connect to.

        Returns:
            bool: flag indicating if the connection attempt was successful
                or not.
        """
        # Send the request and get the response
        response = self.vna_conn_srv(True, ip_addr)

        # Return the response status flag
        return response.status

    def disconnect_from_vna(self):
        """Command the VNA driver to disconnect from instrument.

        Returns:
            bool: flag indicating if the disconnection attempt was successful
                or not.
        """
        # Send the request and get the response
        response = self.vna_conn_srv(False, "")

        # Return the response status flag
        return response.status

    def get_vna_calibration_status(self):
        """Get the current calibration status of VNA instrument.

        Returns:
            int: integer value indicating the calibration status of the VNA
                instrument.
        """
        # Send the request and get the response
        response = self.vna_cal_status_srv()

        # Return the calibration status value from response
        return response.calibration_status

    def set_freq_sweep(self, f_start, f_stop, f_points):
        """Command the VNA driver to configure the frequency sweep.

        Args:
            f_start (float): starting sweep frequency that will be configured
                in the VNA device.
            f_stop (float): stop sweep frequency that will be configured in
                the VNA device.
            f_points (float): frequency points that will be configured to be
                sampled in the VNA device.

        Returns:
            bool: flag indicating if the frequency sweep configuration was
                successful or not.
        """
        # Call the frequency sweep setup service and store the response
        response = self.vna_config_freq_sweep_srv(f_start, f_stop, f_points)

        # Return the response status flag
        return response.result

    def get_frequencies(self):
        """Retrieve the VNA frequencies vector.

        Returns:
            bool: flag indicating if the response data is valid or not.
            str: string with the raw response from the VNA instrument.
        """
        # Call the service and store the response
        response = self.vna_get_freq_srv()

        # Return the response status flag and frequency data string
        return response.status, response.freq_data

    def get_trace(self):
        """Retrieve the current VNA trace.

        Returns:
            bool: flag indicating if the response data is valid or not.
            str: string with the raw response from the VNA instrument.
        """
        # Call the service and store the response
        response = self.vna_get_data_srv()

        # Return the response status and trace data string
        return response.status, response.trace_data

    def get_height(self):
        """Retrieve the current antennae height.

        Returns:
            float: value corresponding to the antennae height.
        """
        # Call the service and store the response
        response = self.antennae_height_srv()

        # Return the distance value
        return response.distance
