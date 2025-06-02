Installation
============

To install **RoBits**, use pip:

.. code-block::

        pip install 'robits[all]'

You can use a :ref:`Conda environment` or a virtual environment.  Next, create a
configuration folder and export the environment variable ``ROBITS_CONFIG_DIR``
that points to the folder, e.g. with

   .. code-block:: bash

      mkdir $HOME/robits_user_config
      # export the environment variable to your .bashrc
      echo "export ROBITS_CONFIG_DIR=$HOME/robits_user_config" >> $HOME/.bashrc

See the :doc:`configuration` section for more details.
If you don't want to install all options you can choose one of the following
installation options. Currently, the following options are available:

 * **real**: Includes dependencies for the Franka Panda, Robotiq gripper, and Realsense cameras.
 * **sim**: Includes dependencies for simulation.
 * **remote**: Includes dependencies to connect via running instances over network
 * **ur**: Includes dependencies for the UR robots 
 * **xarm**: Includes dependencies for the xArm robots
 * **dev**: Includes dependencies for development
 * **usd**: To export a dataset to Blender/IsaacLab and render it USD
 * **audio** Includes dependencies for audio/speech modules



Development
-----------

For development, you can install **RoBits** from source:

.. code-block:: bash

        git clone https://github.com/markusgrotz/robits.git
        cd robits
        pip install -e '.[all,dev]'



Conda environment
-----------------

#. Install conda if not already available. You can use the installer script
   ``scripts/40_install_miniconda.sh``.

#. Create a new conda environment and install other dependencies. 

   .. code-block:: bash
   
      conda create -n robits python=3.10
      # the following dependencies are necessary if you are using a panda robot
      conda install pybind11 poco=1.11.0  gcc make cmake gxx=9.5.0 -c conda-forge

#. Install **RoBits** with ``pip install robits[all]``. See section `Installation`_ or `Development`_ for details.

#. Setup a configuration folder and export the environment variable ``ROBITS_CONFIG_DIR`` that points to the folder, e.g. with

   .. code-block:: bash
          
      mkdir $HOME/robits_user_config
      # export the environment variable to your .bashrc
      echo "export ROBITS_CONFIG_DIR=$HOME/robits_user_config" >> $HOME/.bashrc
