:py:mod:`joint_effort_dynamic_reconfigure_server`
=================================================

.. py:module:: joint_effort_dynamic_reconfigure_server

.. autoapi-nested-parse::

   Small node that spins up a dynamic reconfigure server that can be used to change the
   joint efforts.



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   joint_effort_dynamic_reconfigure_server.JointEffortDynamicReconfigureServer




Attributes
~~~~~~~~~~

.. autoapisummary::

   joint_effort_dynamic_reconfigure_server.effort_dyn_server


.. py:class:: JointEffortDynamicReconfigureServer


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



.. py:data:: effort_dyn_server

   

