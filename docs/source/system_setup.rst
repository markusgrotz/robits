System and Robot setup
======================

This guide walks through installing and setting up **RoBits**.


.. warning::

   It is highly recommended to use a real-time (RT) kernel as otherwise the 
   time-sensitive control tasks may fail.


Single machine setup
--------------------


1. Make sure that the system has booted the real-time kernel by running ``uname -a``. 
   To install a real-time kernel you can execute the script ``scripts/10_install_rt.sh``

2. *(Optional)* You can install the Nvidia drivers while also using the
   RT preempt patched kernel. This is a little bit hacky, but seems to work.
   Either execute ``scripts/30_install_nvidia_dkms.sh`` or install it manually.
   See :ref:`Nvidia DKMS` for further details.



Multi machine setup
-------------------

The following assumes that you have at least two different machines, one for
control with a RT preempt patched kernel and the other machine used for
inference with a GPU.
Configure networking for inter-machine communication. Ensure:

- Both machines are on the same subnet or can reach each other.
- Use a wired connection for lower latency and greater stability.
- Control PC: Install the real-time kernel on this machine. See
  ``scripts/10_install_rt.sh`` and :ref:`Real-time Kernel`

- Perception PC: Install the Nvidia drivers. You can use the ones that are
  shipped with your distribution

   
Next install **RoBits** on both machines. Start the service on the Control PC with ``rb service start <name>``.
For example, ``rb service start robot_panda_real``. Then copy and edit the `robot_client` configuration. 
Use this configuration on the Perception PC to connect to the Control PC. You can test it with a simple command such
as ``rb move up --robot-name robot_client``.


Real-time Kernel
----------------

See the `Kernel wiki <https://wiki.linuxfoundation.org/realtime/preempt_rt_versions>`_ for 
actively maintained PREEMPT_RT versions that match your current kernel version.
See the script  ``scripts/20_install_rt_patch.sh`` for more details.


Nvidia DKMS
-----------

.. note::

   Installing the Nvidia driver with DKMS is only required if you have a RT
   patched kernel.

If you are using a RT preempt patched kernel it is possible to install the
Nvidia driver by specifying the ``IGNORE_PREEMPT_RT_PRESENCE=1`` flag (`See this
gist <https://gist.github.com/pantor/9786c41c03a97bca7a52aa0a72fa9387>`_).

Here is an alternative method using the Dynamic Kernel Module Support (DKMS).
DKMS is a framework to automatically rebuild and install kernel modules when
the kernel is updated.
You can also use the script ``scripts/30_nvidia_dkms.sh``

Install the linux-headers metapackage for your kernel flavour and the Nvidia DKMS package with

.. code-block:: bash

        export NVIDIA_DRIVER_VERSION=560
        sudo apt install linux-headers-$(uname -r)
        sudo apt install nvidia-dkms-${NVIDIA_DRIVER_VERSION}${SERVER}


Then build the module and load it 

.. code-block:: bash

        cat <<EOF | sudo patch /usr/src/nvidia-560.35.03/conftest.sh
        --- a/conftest.sh
        +++ b/conftest.sh
        @@ -716,8 +716,9 @@
         if [ -n "\$IGNORE_PREEMPT_RT_PRESENCE" ]; then
        -    exit 0
        +    echo Skipping preempt RT check
         fi
        EOF

        sudo sed -i '/if \[.*\$IGNORE_PREEMPT_RT_PRESENCE.*\]/,/fi/{
            s/^ *exit 0 *$/    echo Skipping preempt RT check/
        }' /usr/src/nvidia-560.35.03/conftest.sh


        dkms status
        sudo dkms install -m nvidia/560.35.03 -j 12
        sudo depmod -a
        sudo modprobe nvidia


Verify that everything is working with ``uname -a`` and ``nvidia-smi``. You should be able to see something like this

.. code-block:: bash

   markus @ computer ➜  sam2act git:(master) ✗  uname -r                                                                                                                                     [2025-05-28 17:17:06]
   6.8.0-59-generic

   markus @ computer ➜  sam2act git:(master) ✗  nvidia-smi | head -n4                                                                                                                        [2025-05-28 17:19:43]
   Wed May 28 17:19:50 2025       
   +-----------------------------------------------------------------------------------------+
   | NVIDIA-SMI 560.35.03              Driver Version: 560.35.03      CUDA Version: 12.6     |
   |-----------------------------------------+------------------------+----------------------+

Franka Panda setup
------------------
Install the robot according to the manual. By default the webinterface of the
Franka Panda is reachable at `<https://172.16.0.2>`_. If you are using a
different IP address make sure to update the configuration.
If you want to use the CLI to unlock/lock the robot you need to specify the
username and password in the configuration or create an `rselab` user with the
`robot's web interface <https://172.16.0.2/admin/users>`_ by clicking the "+"
button.

Currently two different grippers are supported, the Franka default and Robotiq.
Specify the one you are using in the robot's configuration with the `gripper`
keyword.  If you are using a gripper other than the default make sure that you
select the current end-effector `other` in the
webinterface in the `end-effector <https://172.16.0.2/admin/endeffector>`_
section and add the correct weight for compliance control.



Camera calibration
-------------------
If you are using an Intel RealSense camera you need to extract the intrinsic parameters.
You can use ``rb camera calibrate intrinsics`` for this step. 
To calibrate the extrinsic parameters run ``rb camera calibrate extrinsics``.
Select the camera in the drop down menu and press "Connect" button.  To refine
an existing calibration use the sliders to adjust the parameters and then press
"Save" button.
You can show the existing camera calibration either with ``rb config show`` or
``rb camera info extrinsics/intrinsics``.
You can display the current point cloud with ``rb camera point-cloud view`` as
a sanity check. The coordinate system should be in the robot's base with x-axis
facing front.
