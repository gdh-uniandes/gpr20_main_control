"""Data processing handler module for GPR-20 control."""

import rospy
from gpr20_msgs.srv import ProcessingStoreData


class DataProcessing(object):
    """Class to abstract the data processing utilities access.

    This class is intended to provide methods that ease the process of
    calling data processing utilities for the GPR-20 robot. The class will
    store the attributes required to call such utilities while encapsulating
    them.

    Attributes:
        data_storage_srv (rospy.ServiceProxy): service to call the data
            storage service from the 'GPR-20 Data Processing' utility.
    """

    def __init__(self):
        """Initialize the communication mechanisms for data processing."""
        # Create the proxy to call the processing and store data service
        self.data_storage_srv = rospy.ServiceProxy(
            "processing_store_data",
            ProcessingStoreData
        )

    def store_data(self, survey_dir, x_coord, y_coord, z_coord,
                   antennae_height, timestamp, survey_id, sample_id, vna_freq,
                   vna_trace):
        """Call the processing and store data service.

        The data processing and store service requires the full data stack in
        order to appropiately execute the pipeline. For this reason, this
        method requires the whole data to passed as parameters to then call
        the service.

        Args:
            survey_dir (str): folder in which the data will be stored in. The
                path is relative to the main data folder.
            x_coord (float): coordinate for the X axis in which the data was
                acquired.
            y_coord (float): coordinate for the Y axis in which the data was
                acquired.
            z_coord (float): coordinate for the Z axis in which the data was
                acquired.
            antennae_height (float): antennae height as measured in the
                location in which the data was acquired.
            timestamp (str): timestamp of the data acquisition process.
            survey_id (str): survey identifier. This is the same for every
                sample in a survey.
            sample_id (str): sample identifier. This is unique for each sample
                taken by the robot.
            vna_freq (str): string with the raw response of the VNA when
                requested of the frequency values in which data was acquired.
            vna_trace (str): string with the raw response of the VNA when
                requested of the acquired trace data.
        """
        # Call the service with the received parameters
        self.data_storage_srv(survey_dir, x_coord, y_coord, z_coord,
                              antennae_height, timestamp, survey_id,
                              sample_id, vna_freq,
                              vna_trace)
