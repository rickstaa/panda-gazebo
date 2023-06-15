:py:mod:`joint_position_dynamic_reconfigure_server`
===================================================

.. py:module:: joint_position_dynamic_reconfigure_server

.. autoapi-nested-parse::

   Small node that spins up a dynamic reconfigure server that can be used to change the
   joint positions.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   joint_position_dynamic_reconfigure_server.JointPositionDynamicReconfigureServer




Attributes
~~~~~~~~~~

.. autoapisummary::

   joint_position_dynamic_reconfigure_server.position_dyn_server


.. py:class:: JointPositionDynamicReconfigureServer


   .. py:method:: callback(config, level)

      Dynamic reconfigure callback function.

      :param config: The current dynamic
                     reconfigure configuration object.
      :type config: :obj:`dynamic_reconfigure.encoding.Config`
      :param level: Bitmask that gives information about which parameter has been
                    changed.
      :type level: int

      :returns:

                Modified dynamic reconfigure
                    configuration object.
      :rtype: :obj:`~dynamic_reconfigure.encoding.Config`



.. py:data:: position_dyn_server

   

