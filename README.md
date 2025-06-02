# RoBits - Bits and Bytes for Robotic Manipulation

## `from robits import ♥`

[![Supported Python Versions](https://img.shields.io/pypi/pyversions/robits)](https://pypi.org/project/robits/) 
[![PyPI version](https://img.shields.io/pypi/v/robits)](https://pypi.org/project/robits/) 
[![License](https://img.shields.io/pypi/l/robits)](https://github.com/markusgrotz/robits/LICENSE.md)
[![Code style](https://img.shields.io/badge/code%20style-black-black)](https://black.readthedocs.io/en/stable/)


RoBits is a lightweight, modular and scalable software stack for AI-driven
robotic manipulation.  It is designed for seamless integration with robotic
manipulation policies, by providing essential tools for perception and robotic
control.

RoBits features an intuitive command-line interface to get started quickly.
It's user-friendly and ensures adaptability, efficiency, and ease of use across
diverse robotic applications to help with research experiments.

If you are using this work or find it otherwise useful please cite:
```
M. Grotz, M. Shridhar, Y. Chao, T. Asfour and D. Fox
PerAct2: Benchmarking and Learning for Robotic Bimanual Manipulation Tasks.
https://doi.org/10.48550/arXiv.2407.00278
```

Also consider citing additional resources if necessary (See [Acknowledgement](#Acknowledgement))


![Logo](https://raw.githubusercontent.com/markusgrotz/robits/master/docs/source/_static/logo.png)


## Quickstart

RoBits comes with some default configurations, for example for the Franka Panda
robot. Run `pip install 'robits[all]'`. This will install necessary dependencies and
provide an entry point command `rb`. You can use `rb` to get a list of
commands. For example `rb move home` moves the robot to a default joint
position.

## Command-Line-Interface

Once you have installed `RoBits` you can access the CLI with `rb` in your shell:
```bash
(robits-py3.8) markus @ sockeye ➜  ~  rb                  [2025-02-23 19:53:43]
                                                                                
 Usage: rb [OPTIONS] COMMAND [ARGS]...                                          
                                                                                
 RoBits command line interface for robotic manipulation.                        
 This is the main entry point for all RoBits CLI commands. It provides global   
 verbosity controls and sets up logging configuration.                          
 Use --verbose/-v to increase log detail level (can be used multiple times).    
 Use --quiet/-q to suppress all but error messages.                             
 Example: rb -vv move home --robot-name robot_panda_sim                         
                                                                                
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --verbose  -v  INTEGER RANGE  Increase verbosity of the logging output. Can  │
│                               base used multiple times                       │
│ --quiet    -q                 Suppress all logging output except critical    │
│                               and error messages                             │
│ --help                        Show this message and exit.                    │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ camera    Commands for working with cameras and visual data.                 │
│ config    Commands for viewing and managing system configurations.           │
│ dataset   Dataset related commands                                           │
│ gripper   Gripper related commands                                           │
│ info      Read various information about the current robot state             │
│ move      Commands for moving the robot in different directions.             │
│ panda     Franka Panda related commands                                      │
│ shell     Creates an interactive shell. Use robot variable to interact       │
│ speech    Audio related commands                                             │
╰──────────────────────────────────────────────────────────────────────────────╯
```
To learn more about a command you can write for example `rb dataset --help`,
which will list the subcommands that are related to dataset.


### Configuration

You can display available config files with `rb config list`. To show the
actual content use `rb config show <config_name>` where `config_name`
specifies the name of the config file, e.g., `rb config show robot_panda_real`.
You can specify a user configuration by exporting `ROBITS_CONFIG_DIR`. E.g. by adding
```bash
export ROBITS_CONFIG_DIR=/home/markus/robits_config
```
to your `.bashrc`.
This allows you to specify additional configurations for robots, cameras,
grippers.  For example, you can change the serial number of the camera or the
ip address of a robot.
Once you set your user confiuration folder you can copy and modify an existing configuration with:
```bash
rb config copy
```
Select the configuration you want to modify and open it with your favorite text editor


#### Robot setup

Best practice is to install a RT (Real-Time) kernel. See the script for more details.
With some patches, you can install the NVIDIA driver and access the GPU through
PyTorch on a real-time kernel system. This avoids unnecessary networking calls
and removes the burden of deploying the software as everything can be run from a
single repository.
Once that is done you can get the robot pose with:
`rb info pose` 
To move the robot, use one of the `rb move` commands, e.g.:
`rb move up`

#### Camera setup

To test your camera setup you can use:
`rb camera view` and `rb camera point-cloud view`
For calibration to get the extrinsics, you can use:
`rb camera calibrate extrinsics`
This will launch a TUI. Select the camera and press "connect". Then use the
sliders to adjust the camera pose. Press "save" to store the camera calibration
to your user config directory, which is set by the environment variable
`ROBITS_CONFIG_DIR`.

#### Gripper setup

Basic commands for the gripper are `rb gripper open` or `rb gripper close`.
Please note that there are some limitations with the Franka Panda gripper.
Basically, the robot stops when the gripper is active. See the implementation
for details.

### Data Collection and Replay

To replay collected data, use `rb dataset replay`. You can specify the path, the control method, and the robot name:
```bash
rb dataset replay --input-path ~/data/demo_0000/ --robot-name robot_panda_sim --control-method position
```


## Libraries Acknowledgement

- Libraries
  - [NumPy](https://numpy.org), [SciPy](https://scipy.org/), [Open3D](https://www.open3d.org/), and more
- Real robots and grippers
  - [libfranka](https://github.com/frankaemika/libfranka)
  - [Franky](https://github.com/TimSchneider42/franky)
  - pyRobotiqGripper originally from [castetsb/pyRobotiqGripper](https://github.com/castetsb/pyRobotiqGripper)
  - [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
  - [Python URx](https://github.com/SintefManufacturing/python-urx)
- Simulation
  - [MuJoCo](https://mujoco.org/)
  - [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) and [Robot Descriptions](https://github.com/robot-descriptions/robot_descriptions)
  - [MuJoCo Controllers](https://github.com/kevinzakka/mjctrl)
