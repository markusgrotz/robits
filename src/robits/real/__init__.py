"""
Implementation of the RoBits framework for real physical hardware.

This package provides concrete implementations of the RoBits interfaces for
controlling real robots, grippers, and cameras. It handles the complexities
of communicating with physical hardware devices through their respective APIs.
Robot implementation are grouped in a plugin style.

Key components:

- Robot: Implementation for Franka Panda, XArm, UR robots
- Control: Position and Cartesian control implementations
- Grippers: Implementations for Franka, Xarm and Robotiq grippers
- Cameras: Implementation for RealSense cameras

Note: Most components require specific hardware configuration and network
connectivity to function properly. Please see the documentation
"""
