"""Module containing several custom exceptions."""


class InputMessageInvalidError(Exception):
    """Custom exception that is thrown when the input message to a 'panda_gazebo'
    service or action is invalid.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initialise InputMessageInvalidError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details: Additional keyword arguments that can be used to supply the user
                with more details about why the exception occurred. These details are
                stored in the ``details`` attribute.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details


class JointLimitsInvalidError(Exception):
    """Custom exception that is thrown when the joint limits specified by the user are
    not within the joint limits of the robot.

    Attributes:
        log_message (str): The full log message.
        details (dict): Dictionary containing extra Exception information.
    """

    def __init__(self, message="", log_message="", **details):
        """Initialise JointLimitsInvalidError exception object.

        Args:
            message (str, optional): Exception message specifying whether the exception
                occurred. Defaults to ``""``.
            log_message (str, optional): Full log message. Defaults to ``""``.
            details: Additional keyword arguments that can be used to supply the user
                with more details about why the exception occurred. These details are
                stored in the ``details`` attribute.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details
