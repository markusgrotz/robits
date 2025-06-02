Frequently asked questions
==========================


.. contents::

Does it run on macOS?
---------------------
Partial support is available on macOS.  The issue here is that **MuJoCo
simulation** and other components require the visualization to be executed in the main thread.
For full functionality, Linux is
recommended. To run **RoBits** on macOS you can start it with ``mjpython``. To edit the entry point you can use the following command:

.. code-block:: bash

   sed -i -e 's/\/python[0-9.]*$/\/mjpython/' "$(whereis rb | awk '{print $2}')"

Unfortunately, doing so will break the Open3D visualization viewer.

Does it work with an older Franka?
----------------------------------
You need to downgrade libfranka. For the correction version see `compatibility
with libfranka <https://frankaemika.github.io/docs/compatibility.html>`_.
Additionally, you need to compile franky. Contact us for prebuild Python
packages.
The following scripts ``scripts/60_install_libfranka_0.8.0.sh`` and ``scripts/65_install_franky.sh``  will help you to install a different libfranka version.


How can I change the speed of the robot?
----------------------------------------
Change ``dynamics_factor`` parameter in your robot configuration. 

.. code-block:: JSON
   :emphasize-lines: 3

    {
        "robot_name": "panda",
        "dynamics_factor": 0.2,
        "class_path": "robits.real.franka.robot.Franka",
    }


Can it be used in simulation only?
----------------------------------
Yes. The stack can operate entirely in simulation using **MuJoCo**, which provides high-performance physics simulation for robotic manipulation tasks.



How can I change the user for the Franka robot?
-----------------------------------------------
For ``rb franka unlock/lock`` you need to add  ``main.json`` in your ``ROBITS_CONFIG_DIR`` with the following values:

.. code-block:: JSON
   :emphasize-lines: 2-6

    {
        "franka_web": {
            "ip_addr": "172.16.0.2",
            "user": "your-user-name",
            "password": "your-secret-password"
        }
    }