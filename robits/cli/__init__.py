"""
Command-line interface for the RoBits robotic manipulation library.

This package provides a comprehensive CLI for controlling robots, cameras,
and grippers, as well as managing datasets and configurations. The CLI is
built on Click with Rich enhancements for a modern terminal experience.

Key command groups:

 * robot: General robot information and control
 * move: Robot movement operations
 * camera: Camera control and visualization
 * gripper: Gripper control
 * dataset: Dataset management and replay
 * config: Configuration management
 * speech: Speech recognition and synthesis


Usage:

.. code-block::

    $ rb --help                    # Show main help
    $ rb move --help               # Show move command help
    $ rb move up --robot-name ...  # Move robot upward
"""
