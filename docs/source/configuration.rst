Configuration
=============

Configurations are a cornerstone of **RoBits**.  As your robot setup might differ
you need to change existing configurations.  Basically, there are two different
configuration types 1.) System and 2.) User. The former are provided with the
package, while the latter can be changed by the user.
The user configuration supersedes system configuration.  You can point to a
different user configuration folder by exporting the environment variable
called ``ROBITS_CONFIG_DIR``


.. code-block:: bash
  
        mkdir $HOME/robits_user_config
        # export the environment variable to your .bashrc
        echo "export ROBITS_CONFIG_DIR=$HOME/robits_user_config" >> $HOME/.bashrc


By changing the environment variable you can point to different
configuration folders.  To interact with the configuration you can use the
command line interface ``rb config``.  For example, to copy an existing
configuration you use ``rb config copy``.  The command ``rb config edit`` is a
convenient way to edit a config with your current system editor that is specified by
``$EDITOR``.

Configurations are stored in JSON. The idea is that the class is automatically
instantiated and parameters are passed.  Here the "class_path" specifies which
package to import and class to instantiate. For `robot` configuration the
`gripper`, `cameras` or `camera` point to other configuration files and are
automatically instantiated recursively.  


.. code-block:: JSON

    {
        "robot_name": "panda",
        "gripper": "gripper_robotiq_real",
        "camera": "camera_front_real",
        "default_joint_positions": [
            0.000,
            -0.7834,
            0.000,
            -2.3579,
            0.000,
            1.5708,
            -1.5
        ],
        "class_path": "robits.real.robot.franka.Franka",
        "ip_addr": "172.16.0.2"
    }


Please see the :doc:`api/robits` section or refer to the actual implementation for details on the parameters.


Default
-------

When using the command line interface you can set a default robot configuration with the environment variable ``ROBITS_DEFAULT_ROBOT``


.. code-block:: bash

        echo "export ROBITS_DEFAULT_ROBOT=robot_panda_sim" >> $HOME/.bashrc
        
.. code-block:: python
    
    import rich_click as click
    from robits.cli import cli_options

    @click.command
    @cli_options.robot()
    def cli(robot):
        print(robot)
        # Implement your robot code here

    if __name__ == "__main__":
        cli()





Remote
------
**RoBits** has the ability to launch remote services and connect to them using ZeroMQ.
This is done by wrapping the configuration name around a remote instance.
To connect use

.. code-block:: JSON

    {
        "class_path": "robits.remote.client.robot.RobotZMQClient",
        "address": "localhost",
        "robot_name": "remote"
    }