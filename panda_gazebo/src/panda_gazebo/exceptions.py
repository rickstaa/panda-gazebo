"""Module containing several custom exceptions."""


class InputMessageInvalidError(Exception):
    """Custom exception that is raised when an input argument to one of the
    panda_gazebo functions is invalid.

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
            details (dict): Additional dictionary that can be used to supply the user
                with more details about why the exception occurred.
        """
        super().__init__(message)

        # Set attributes.
        self.log_message = log_message
        self.details = details
