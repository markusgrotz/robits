Data collection
===============

RoBits support kinesthetic teaching, teleoperation as well as scripted
demonstrations.  
The easiest way is to start the robot app and toggle the "Kinesthetic Teaching" mode.
Now you can move the robot arm to the desired position. Close the gripper using the
"Gripper" toggle button.  Press the "Add Waypoint" button to save the current pose of the robot as waypoints.
Once you are done, press the "Save" button to serialize all current waypoints. 
You can restore the state with "Load" button.  Press "Replay" button to execute the  currently loaded waypoints.  
Make sure to complete the :doc:`system_setup` and the :doc:`configuration` instructions.

.. note::
   More available soon


Collecting data is quite simple and can be done by using a ``with`` statement.
Given a ``robot`` instance and a ``output_path`` you can create a dataset with:

.. code-block:: python

   from robits.dataset.io.recorder import DatasetRecorder
   from robits.dataset.io.writer import DatasetWriter

    with DatasetWriter(output_path, DatasetRecorder(robot)):

        print("Move robot") 


See the :py:class:`robits.dataset.io.writer.DatasetWriter` class for more details.

Scripted data collection
------------------------

Here is an example on how collect proprioception data.

.. literalinclude:: ../../examples/simple_data_collection.py
   :linenos:


Stack blocks
------------

When interacting with the environment it is necessary to get a grasp pose.
Specifying grasp points with an interactive gui is quite fast.

.. literalinclude:: ../../examples/stack_blocks.py
   :linenos:


