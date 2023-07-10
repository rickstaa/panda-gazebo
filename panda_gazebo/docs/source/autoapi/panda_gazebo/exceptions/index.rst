:py:mod:`panda_gazebo.exceptions`
=================================

.. py:module:: panda_gazebo.exceptions

.. autoapi-nested-parse::

   Module containing several custom exceptions.



Module Contents
---------------

.. py:exception:: InputMessageInvalidError(message='', log_message='', **details)


   Bases: :py:obj:`Exception`

   Custom exception that is raised when an input argument to one of the
   panda_gazebo functions is invalid.

   .. attribute:: log_message

      The full log message.

      :type: str

   .. attribute:: details

      Dictionary containing extra Exception information.

      :type: dict


