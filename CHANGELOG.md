# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

-

## [0.6.1] - 2026-01-23

- Drop Python 3.9 support
- Add support for webcams
- Protect robotiq gripper position read with lock
- Minor code refactoring and docs updates

## [0.6.0] - 2025-12-16

 - Added gripper.set_pos()
 - Simulation modules revised: blueprints, camera, env, env_design, model_factory, utils.
 - Fix mujoco dependency version for Python 3.9

## [0.5.2] - 2025-12-15

 - Migrate from poetry to PEP 621 project metadata
 - Refactored code
 - Update pyrobotiqgripper dependency
 - Revised package release workflow

## [0.5.1] - 2025-06-10

 - Revised MuJoCo model_factory handling of objects with freejoints
 - Fix wrist camera extrinsics 
 - Updated documentation
 - Moved SAM2Act agent to sam2act repository 
 - Fixed gripper camera extrinsics
 - Remove mamba from scripts


## [0.5.0] - 2025-06-02

 - Initial release


## Release history

[Unreleased]: https://github.com/markusgrotz/robits/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/markusgrotz/robits/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/markusgrotz/robits/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/markusgrotz/robits/compare/v0.5.2...v0.6.0
[0.5.2]: https://github.com/markusgrotz/robits/compare/v0.5.1...v0.5.2
[0.5.1]: https://github.com/markusgrotz/robits/compare/v0.5.0...v0.5.1
