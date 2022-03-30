

from re import S
import rospy
from gpr20_msgs.msg import ControlFeedback
from std_srvs.srv import Empty, EmptyResponse
from gpr20_control.survey_thread import SurveyThread
from gpr20_msgs.srv import ControlSurvey, ControlSurveyResponse


class Control(object):
    """ Control class definition for GPR-20 robot.

    """

    def __init__(self):

        # Initializes the control node
        rospy.init_node("gpr20_control")

        # Define the survey service
        rospy.Service(
            "execute_survey",
            ControlSurvey,
            self.__execute_survey_handler
        )

        # Define the stop survey service
        rospy.Service(
            "stop_survey",
            Empty,
            self.__stop_survey_handler
        )

        # Creates the attribute to store survey thread
        self.survey_thread = None

        # Define the control feedback topic
        self.control_fb_pub = rospy.Publisher(
            "control_feedback",
            ControlFeedback,
            queue_size=100
        )

        # Define a rate to publish the feedback
        rate = rospy.Rate(10)

        # Iterate until node is shutdown
        while not rospy.is_shutdown():

            # Call the method to publish the feedback
            self.__publish_feedback()

            # Sleep according to defined rate
            rate.sleep()

    def __execute_survey_handler(self, srv):
        
        rospy.loginfo("Received survey request!")

        # Check if survey thread is running
        if self.survey_thread is not None:

            # Check if thread is alive
            if self.survey_thread.is_alive():

                # Return the service indicating that a survey is taking place
                return ControlSurveyResponse(
                    False,
                    "There is a survey taking place!"
                )
        
        # Create the survey parameters dictionary
        survey_params = {
            "vna_ip": srv.vna_ip,
            "x_start": srv.x_start,
            "x_stop": srv.x_stop,
            "x_points": srv.x_points,
            "y_start": srv.y_start,
            "y_stop": srv.y_stop,
            "y_points": srv.y_points,
            "f_start": srv.f_start,
            "f_stop": srv.f_stop,
            "f_points": srv.f_points,
        }

        # Instantiate the survey thread
        self.survey_thread = SurveyThread(survey_params)

        # Start the thread execution
        self.survey_thread.start()

        # Return response indicating that survey initialized
        return ControlSurveyResponse(
            True,
            "Survey has initialized!"
        )

    def __stop_survey_handler(self, req):
        """Handler to """
        # Check if thread exists
        if self.survey_thread is not None:

            # Check if thread is alive
            if self.survey_thread.is_alive():

                # Set the stop flag
                self.survey_thread.set_stop_flag()

        # Return the empty response
        return EmptyResponse()

    def __publish_feedback(self):
        
        # Check if survey thread is running
        if self.survey_thread is not None:

            # Define the survey status
            on_survey = self.survey_thread.is_alive()

            # Get the status message
            status = self.survey_thread.status()

            # Get visited points
            visited = self.survey_thread.current_point

            # Get remaining points
            remaining_points = self.survey_thread.remaining_points

            # Get completion
            completion = self.survey_thread.completion

            # Get remaining time
            remaining_time = self.survey_thread.remaining_time   

            # Publish the feedback
            self.control_fb_pub(on_survey, status, visited, remaining_points,
                completion, remaining_time)
