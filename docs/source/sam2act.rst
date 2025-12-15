Memory-based manipulation with SAM2Act
======================================

.. note:: 
        This software was modified after running the experiments. 
        Functionality should still be the same.

The following covers inference and data collection with a Franka Emika Panda,
Robotiq gripper, and an Intel Realsense camera.  Instructions for other robots
might differ.  For example, if you use the original Franka gripper you need to
change the configuration for the robot.  Please see the paper and the
`project webpage <https://sam2act.github.io/>`_ for more details.


Installation
------------

Installation of **SAM2Act** is covered in the `SAM2Act code repository <https://github.com/sam2act/sam2act>`_. 
See the section `Environment setup <https://github.com/sam2act/sam2act?tab=readme-ov-file#environment-setup>`_ in the Readme.
You can install the extras "real" for sam2act with ``pip install -e '.[real]'``.
This will automatically install **RoBits**  and other dependencies.
For the robot setup see the :doc:`system_setup` section for details on the
installation. Especially, check the camera calibration with ``rb camera
calibrate extrinsics``. If necessary, adjust the current camera calibration by
adjusting the sliders.

Inference
---------

1. Make sure that the system has booted the real-time kernel by running ``uname -a``.
   See :doc:`system_setup` for more details.

2. Ensure that the robot is safe to operate.  The default configuration
   assumes that the robot is reachable on `172.16.0.2`, which is the factory
   default. Either create another user in Franka's webinterface or update the
   configuration if you want to use the cmd to unlock the robot.  Turn the
   robot on and unlock it with ``rb panda unlock``. 

3. Toggle the e-stop. You can test your robot setup with ``rb info pose`` or ``rb move up``

4. Load the weights and run the inference with the entry point
   ``sam2act-agent`` provided by sam2act. You can also specify the command options so you won't be
   prompted.

    .. code-block:: bash
    
            sam2act-agent --robot-name "robot_panda_real" --execution-mode auto --instruction "turn on the lamp" --model-path /home/markus/models_sam2act/model_real_lamp_1/model_9.pth
    

Training
--------

.. note::

   Documentation and best practice for real-world training will be available soon



Since the keyframe heuristic relies on pauses it is necessary to record and
replay the trajectory or use scripted demonstrations. See
:doc:`data_collection` for more details.


References
----------

```
SAM2Act: Integrating Visual Foundation Model with A Memory Architecture for Robotic Manipulation.
Haoquan Fang, Markus Grotz, Wilbert Pumacay, Yi Ru Wang, Dieter Fox, Ranjay Krishna, Jiafei Duan
International Conference on Machine Learning (ICML), 2025.
```