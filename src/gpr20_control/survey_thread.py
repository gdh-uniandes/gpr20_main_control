

import time
from sys import exit
from uuid import uuid4
from threading import Thread
from datetime import datetime

from gpr20_control.planning import Planning
from gpr20_control.cartesian_arm import CartesianArm
from gpr20_control.data_processing import DataProcessing
from gpr20_control.data_acquisition import DataAcquisition


class SurveyThread(Thread):

    def __init__(self, survey_params):
        """Initialize the survey thread with the survey parameters.
        
        Args:
            survey_params (dict): 
        """
        super(SurveyThread, self).__init__()

        # Intialize the X coordinate parameters
        self.x_start = survey_params['x_start']
        self.x_stop = survey_params['x_stop']
        self.x_points = survey_params['x_points']
        
        # Calculate the delta for X coordinates
        self.x_delta = Planning.calculate_delta(
            self.x_start,
            self.x_stop,
            self.x_points
        )

        # Intialize the Y coordinate parameters
        self.y_start = survey_params['y_start']
        self.y_stop = survey_params['y_stop']
        self.y_points = survey_params['y_points']
        
        # Calculate the delta for Y coordinates
        self.y_delta = Planning.calculate_delta(
            self.y_start,
            self.y_stop,
            self.y_points
        )

        # Calculate the total points number
        self.total_points = self.x_points * self.y_points * 2

        # Initialize the VNA freq sweep parameters
        self.f_start = survey_params['f_start']
        self.f_stop = survey_params['f_stop']
        self.f_points = survey_params['f_points']

        # Initialize the VNA IP address parameter
        self.vna_ip = survey_params['vna_ip']

        # Initialize the current point attribute
        self.current_point = 0

        # Initialize the remaining points attribute
        self.remaining_points = 0

        # Initialize the completion attribute
        self.completion = 0.0

        # Initialize the completion attribute
        self.remaining_time = 0.0

        # Initialize the stop flag attribute
        self.stop_flag = False

        # Initialize the status attribute
        self.status = "Thread was initialized!"

        # Initialize the data acquisition handler
        self.data_handler = DataAcquisition()

        # Initialize the Cartesian arm handler
        self.cartesian_handler = CartesianArm()

        # Initialize the data processing handler
        self.processing_handler = DataProcessing()


    def run(self):
        """Define the execution workflow of the survey thread.
        
        The execution flow starts by configuring the VNA device for the
        survey. The first step is to command the connection to the VNA
        instrument. Then, the calibration status of the instrument is
        retrieved to ensure the data acquisition quality. Finally, the
        frequency sweep is configured in the device.
        
        """
        # Attempt to connect to the VNA
        vna_conn_result = self.data_handler.connect_to_vna(self.vna_ip)

        # Check the VNA connection attempt
        self.__check_flag(
            vna_conn_result,
            "VNA connection was successful!",
            "VNA connection was not successful!"
        )

        # Check the stop flag status
        self.__check_stop_flag()

        # Retrieve the VNA calibration status
        vna_cal_status = self.data_handler.get_vna_calibration_status()

        # Convert the VNA calibration status to boolean flag
        vna_cal_status = True if vna_cal_status >= 2 else False

        # Check the VNA calibration status
        self.__check_flag(
            vna_cal_status,
            "VNA calibration status is OK!",
            "VNA calibration status is not correct!"
        )

        # Check the stop flag
        self.__check_stop_flag()

        # Configure the VNA with the frequency sweep parameters
        vna_freq_setup_status = self.data_handler.set_freq_sweep(
            self.f_start,
            self.f_stop,
            self.f_points
        )

        # Check if the VNA frequency sweep configuration was successful
        self.__check_flag(
            vna_freq_setup_status,
            "VNA frequency sweep was successfuly configured!",
            "An error occured while configuring the VNA frequency sweep!"
        )

        # Check the stop flag
        self.__check_stop_flag()

        # Get the VNA freq vector
        freq_response = self.data_handler.get_frequencies()

        # Check the responses contents
        self.__check_flag(
            freq_response[0],
            "Frequencies are OK!",
            "Error while retrieving frequencies!"
        )

        # Get the frequencies
        vna_freq = freq_response[1]

        # Define a flag that indicates that axes have executed homing
        axes_homing_done = False

        # Check the stop flag
        self.__check_stop_flag()

        # Iterate until homing has been done on three axes
        while not axes_homing_done:

            # Get the homing status from Cartesian arm handler
            axes_homing_done = self.cartesian_handler.get_homing_status()

            # Check the stop flag
            self.__check_stop_flag()

        # Define the survey folder
        survey_folder = datetime.now().strftime("%Y%m%d%H%M") + '/'

        # Define the survey ID identifier
        survey_id = str(uuid4())

        # Define a point counter
        self.current_point = 0

        # Create a direction flag for X axis
        x_direction = True

        # Calculate total points
        total_points = self.x_points * self.y_points * 2

        # Check the stop flag
        self.__check_stop_flag()

        # Define the start time
        start_time = time.time()

        # Iterate over Y axis coordinates (slow)
        for y_indx in range(self.y_points):

            # Calculate the Y-axis coordinate
            current_y_coord = Planning.calculate_coord(
                self.y_start,
                self.y_stop,
                y_indx,
                self.y_delta,
                True
            )

            # Iterate over X axis coordinates (fast)
            for x_indx in range(self.x_points):

                # Calculate the X axis coordinate
                current_x_coord = Planning.calculate_coord(
                    self.x_start,
                    self.x_stop,
                    x_indx,
                    self.x_delta,
                    x_direction
                )

                # Iterate twice on Z axis (antennae)
                for z_indx in range(2):

                    # Define the sample ID identifier
                    sample_id = str(uuid4())

                    # Check if it is the first Z axis measurement
                    if z_indx == 0:

                        # Define the current Z axis coordinate
                        current_z_coord = 0.0

                    # Execute if not first Z axis measurement
                    else:

                        # Conditionally assign the Z axis coordinate angle
                        current_z_coord = 90.0 if current_x_coord < 500 else -90.0

                    # Move the Cartesian arm to target position (first measurement)
                    self.cartesian_handler.go_to_coordinates(
                        current_x_coord,
                        current_y_coord,
                        current_z_coord
                    )

                    # Get the anntennane height data
                    antennae_height = 10.0 # self.data_handler.get_height()

                    # Get the traces from VNA device
                    trace_response = self.data_handler.get_trace()

                    # Check the traces response
                    self.__check_flag(
                        trace_response[0],
                        "Trace response is OK!",
                        "Error while retrieving traces response!"
                    )

                    # Get the trace data
                    vna_trace = trace_response[1]

                    # Define the measurement timestamp
                    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

                    # Store the sample
                    self.processing_handler.store_data(
                        survey_folder,
                        current_x_coord,
                        current_y_coord,
                        current_z_coord,
                        antennae_height,
                        timestamp,
                        survey_id,
                        sample_id,
                        vna_freq,
                        vna_trace
                    )

                    # Increment points counter
                    self.current_point += 1

                    # Calculate completion
                    self.completion = self.current_point / total_points

                    # Calculate remaining points
                    self.remaining_points = total_points - self.current_point

                    # Calculate the average time per point
                    avg_time = (time.time() - start_time) / self.current_point

                    # Get the remaining time
                    self.remaining_time = avg_time * self.remaining_points

                    # Set the status
                    self.status = "Current point: {}, Completion: {:06.2f}".format(
                        self.current_point,
                        self.completion * 100.0
                    )

                    # Check the stop flag
                    self.__check_stop_flag()

                # Invert the direction flag
                x_direction = not x_direction

    def set_stop_flag(self):
        """Set the stop flag attribute to halt thread execution."""
        self.stop_flag = True

    def __check_stop_flag(self):
        """ Check the stop flag and exit thread if set."""
        # Check the stop flag status
        if self.stop_flag:

            # Set the status message to indicate that a stop command
            self.status = "Node stopped due to user command."

            # Exit the thread if flag is set
            exit()

    def __check_flag(self, flag, true_msg, false_msg):
        """Checks a status flag to decide thread execution.
        
        Args:
            flag (bool): flag that will be evaluated. The flag must be 'True'
                in order to continue with the thread execution.
            true_msg (str): message that will be assigned to the status
                attribute when flag is 'True'.
            false_msg (str): message that will be assigned to the status
                attribute when flag is 'False'.
        """
        # Check if flag is 'True'
        if flag:

            # Define the status attribute with the message when flag is True
            self.status = true_msg

        # Execute if flag is not 'True'
        else:

            # Define the status attribute with the message when flag is False
            self.status = false_msg

            # Exit the thread execution
            exit()
