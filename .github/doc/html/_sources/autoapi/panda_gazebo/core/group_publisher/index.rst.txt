:py:mod:`panda_gazebo.core.group_publisher`
===========================================

.. py:module:: panda_gazebo.core.group_publisher

.. autoapi-nested-parse::

   Class used to group a number of publishers together.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   panda_gazebo.core.group_publisher.GroupPublisher




.. py:class:: GroupPublisher(iterable=[])


   Bases: :py:obj:`list`

   Used for bundling ROS publishers together and publishing
   to these publishers at the same time.

   .. py:method:: publish(messages)

      Publishes a list of messages to the publishers contained on the
      GroupPublisher object. The index of the message corresponds to the
      publisher the message will be published to.

      :param messages: List containing the messages to be published.
      :type messages: list

      :raises ValueError: If something went wrong during publishing.



