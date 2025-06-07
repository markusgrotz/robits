Examples
========

The following describes some examples for the `Command Line`_ as well as
`Picking up a block`_.  It is helpful to read the :doc:`system_setup` and the
:doc:`configuration` sections.



Command Line
------------

**RoBits** provides entry points for quick interaction. You can use the ``rb``
entry point in your shell. Here are some examples:

.. code-block:: bash

   # Using the speech modules
   rb speech say "Hello World!"

   # Moving the robot to the default pose
   rb move home

   # Closing the gripper
   rb gripper close

   # Opening the robotiq gripper as specified in the gripper_robotiq_real config
   rb gripper open --griper-name gripper_robotiq_real

   # Showing available RealSense cameras
   rb camera list

   # Getting an overview of available configurations
   rb config list

   # Editing the robot_panda_real config
   rb config edit robot_panda_real



Picking up a block
------------------

The following command-line interface script performs a basic robotic pick task by initializing the robot, opening the gripper, executing a series of Cartesian movements 
to approach and grasp an object, lifting it with a relative motion, and returning the robot to its home position. 
The control is context-managed Cartesian control mode (see :obj:`robits.core.abc.control.control_types`)
For more information on CLI setup and options, refer to :mod:`robits.cli.cli_utils` and :func:`robits.cli.cli_options.robot`.
Please note that the grasp pose is relative to the robot. See :func:`robits.utils.transform_utils.transform_pose` to transform it into the 
robot's coordinate system.


.. literalinclude:: ../../examples/pick_up_block.py
   :emphasize-lines: 19,26-31
   :linenos:




Shell integration
-----------------

You can direclty interact with the robot using ``rb shell``. To launch a Mujoco simulation with the panda use ``rb shell --robot robot_panda_sim`` or ``export ROBITS_DEFAULT_ROBOT=robot_panda_sim``.
Note that for Mujoco simulation the environemnt is built once you interact with it. Just access ``robot.env`` in the Ipython shell to trigger a build of the environment.
Next paste the following to pick up a the blue block:


.. code-block:: python

