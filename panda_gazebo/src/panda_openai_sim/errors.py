"""Module containing several custom ROS errors."""


# Main python imports
import sys

from panda_openai_sim.functions import list_2_human_text

# ROS python imports
import rospy


#################################################
# Custom ROS errors #############################
#################################################
def arg_type_error(arg_name, depth, invalid_types, valid_types, shutdown=True):
    """This function displays a argument type invalid ROS error/warning and shutdown the
    ROS node if requested.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    depth : int
        The dict depth at which the error occurred.
    invalid_type : tuple
        The type that was invalid.
    valid_types : tuple
        The types that are valid.
    shutdown : bool, optional
        Whether to shutdown the ROS node after the error has been thrown, by default
        True.
    """

    # Throw Type error and shutdown ROS node
    if shutdown:
        if depth == 0:
            logerr_msg = (
                "Shutting down '%s' since input argument '%s' was of type %s while "
                "the PandaTaskEnv expects it to be of type %s. Please fix the type "
                "and try again."
                % (
                    rospy.get_name(),
                    arg_name,
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in invalid_types],
                        end_seperator="and",
                    ),
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in valid_types],
                        end_seperator="or",
                    ),
                )
            )
        else:
            logerr_msg = (
                "Shutting down '%s' since input argument '%s' contains items that have "
                "type %s while the PandaTaskEnv only expects %s. Please fix the type "
                "and try again."
                % (
                    rospy.get_name(),
                    arg_name,
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in invalid_types],
                        end_seperator="and",
                    ),
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in valid_types],
                        end_seperator="or",
                    ),
                )
            )
        rospy.logerr(logerr_msg)
        sys.exit(0)
    else:  # Throw warning
        if depth == 0:
            logwarn_msg = (
                "Input argument '%s' was of type %s while the PandaTaskEnv expects it "
                "to be of type %s."
                % (
                    arg_name,
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in invalid_types],
                        end_seperator="and",
                    ),
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in valid_types],
                        end_seperator="or",
                    ),
                )
            )
        else:
            logwarn_msg = (
                "Input argument '%s' contains items that have type %s while the "
                "PandaTaskEnv only expects %s."
                % (
                    arg_name,
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in invalid_types],
                        end_seperator="and",
                    ),
                    list_2_human_text(
                        ["'" + str(item.__name__) + "'" for item in valid_types],
                        end_seperator="or",
                    ),
                )
            )
        rospy.logwarn(logwarn_msg)
        sys.exit(0)


def arg_keys_error(arg_name, missing_keys=[], extra_keys=[], shutdown=True):
    """This function displays a argument keys invalid ROS error/warning and shutdown the
    ROS node if requested.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    missing_keys : list, optional
        The dictionary keys that were missing from the input argument, by default
        ``[]``.
    extra_keys : list
        The dictionary keys that were present but should not be, by default ``[]``.
    shutdown : bool, optional
        Whether to shutdown the ROS node after the error has been thrown, by default
        True.
    """

    # Display error/warning if missing or extra keys are not empty
    if missing_keys or extra_keys:

        # Log error message and shutdown if requested
        if shutdown:  # Display error if shutdown is True
            logerr_msg = (
                "Shutting down '%s' since input argument '%s' contains %s. Please %s "
                "and try again."
                % (
                    rospy.get_name(),
                    arg_name,
                    (
                        "missing and invalid keys"
                        if len(missing_keys) > 0 and len(extra_keys) > 0
                        else (
                            (
                                "several missing keys"
                                if len(missing_keys) > 1
                                else "a missing key"
                            )
                            if len(missing_keys) > 0
                            else (
                                "several invalid keys"
                                if len(missing_keys) > 1
                                else "a invalid key"
                            )
                        )
                    ),
                    (
                        "remove [%s] and add [%s]"
                        % (
                            list_2_human_text(
                                ["'" + str(item) + "'" for item in extra_keys],
                                end_seperator="and",
                            ),
                            list_2_human_text(
                                ["'" + str(item) + "'" for item in missing_keys],
                                end_seperator="and",
                            ),
                        )
                        if len(missing_keys) > 0 and len(extra_keys) > 0
                        else (
                            (
                                "remove [%s]"
                                % (
                                    list_2_human_text(
                                        ["'" + str(item) + "'" for item in extra_keys],
                                        end_seperator="and",
                                    )
                                )
                                if len(extra_keys) > 0
                                else "add [%s]"
                                % (
                                    list_2_human_text(
                                        [
                                            "'" + str(item) + "'"
                                            for item in missing_keys
                                        ],
                                        end_seperator="and",
                                    )
                                )
                            )
                        )
                    ),
                )
            )
            rospy.logerr(logerr_msg)
            sys.exit(0)
        else:  # Throw warning
            logwarn_msg = "Input argument '%s' contains %s." % (
                arg_name,
                (
                    "missing and invalid keys"
                    if len(missing_keys) > 0 and len(extra_keys) > 0
                    else (
                        (
                            "several missing keys"
                            if len(missing_keys) > 1
                            else "a missing key"
                        )
                        if len(missing_keys) > 0
                        else (
                            "several invalid keys"
                            if len(missing_keys) > 1
                            else "a invalid key"
                        )
                    )
                ),
            )
            rospy.logwarn(logwarn_msg)


def arg_value_error(arg_name, invalid_values, valid_values, shutdown=True):
    """This function displays a value invalid ROS error/warning and shutdown the ROS
    node if requested.

    Parameters
    ----------
    arg_name : str
        The name of the argument.
    invalid_values : list, str
        The values that were invalid.
    valid_values : list, str
        A list of valid values.
    shutdown : bool, optional
        Whether to shutdown the ROS node after the error has been thrown, by default
        True.
    """

    # Convert string input to list
    if isinstance(invalid_values, str):
        invalid_values = [invalid_values]
    if isinstance(valid_values, str):
        valid_values = [valid_values]

    # Log error message and shutdown if requested
    if shutdown:  # Display error if shutdown is True
        logerr_msg = (
            "Shutting down '%s' since input argument '%s' contains %s %s. Valid "
            "values for the '%s' input argument are %s. Please fix this and try "
            "again."
            % (
                rospy.get_name(),
                arg_name,
                "a invalid value" if len(invalid_values) else "invalid values",
                list_2_human_text(
                    ["'" + str(item) + "'" for item in invalid_values],
                    end_seperator="and",
                ),
                arg_name,
                list_2_human_text(
                    ["'" + str(item) + "'" for item in valid_values],
                    end_seperator="and",
                ),
            )
        )
        rospy.logerr(logerr_msg)
        sys.exit(0)
    else:  # Display warning
        logwarn_msg = (
            "Input argument '%s' contains %s %s. Valid values for the '%s' input "
            "argument are %s."
            % (
                arg_name,
                "a invalid value" if len(invalid_values) else "invalid values",
                list_2_human_text(
                    ["'" + str(item) + "'" for item in invalid_values],
                    end_seperator="and",
                ),
                arg_name,
                list_2_human_text(
                    ["'" + str(item) + "'" for item in valid_values],
                    end_seperator="and",
                ),
            )
        )
        rospy.logwarn(logwarn_msg)
