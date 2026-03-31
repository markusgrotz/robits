# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- Add pocket TTS
- Include all gripper joints for sim.gripper.is_open 
- Fix normalization for gripper joint positions
- Reduce tolerance when checking rotation matrix determinant

## [0.8.2] - 2026-03-20

- Add JSON encoder to handle numpy parameters for remote serialization
- Remote robots now support grippers and cameras

## [0.8.1] - 2026-02-09

- Parse mass property for geom tags when importing MuJoCo XML files
- Parse degree option when importing MuJoCo XML files
- Fix DEFAULT_FREE_JOINT_QPOS handling in model_factory module
- Add more heuristics to detect robot/grippers when importing from MuJoCo XML
- Revise config factory and improve support for bimanual robots

## [0.8.0] - 2026-01-27

- Fix home key extraction and reduce dummy model size
- Revise joint indexing and gripper name prefixing
- Revise gripper initialization and attachment logic in model factory
- Remove dependency versioning for legacy Python 3.9
- Fix and improve mujoco import
- Extracted camera intrinsics into a dedicated utility to avoid unnecessary open3d imports.

## [0.7.2] - 2026-01-23

- Fix MuJoCo importer after blueprint refactor from name to path
- Fix Ruff check warnings

## [0.7.1] - 2026-01-23

- Add robits.sim.converters module
- Format code and clean up

## [0.7.0] - 2026-01-23

- Switch to hierarchical scene Blueprints with path-based grouping and parent lookup.
- Use Blueprint basename as MJCF element name for lookup instead of id.
- Add Pose composition operator (@) with stricter validation and accessors.
- Add pose helpers in robits.sim.mjcf_utils: set_pose, add_offset_pose, pose_from_element, set_object_pose, get_home_key_qpos.
- GeomBlueprint: add mass field; auto-wrap non-static geoms and add a freejoint when needed.
- SceneBuilder: add export_with_assets(...); merge all keyframes into home.
- Update Attachment: gripper_path/wrist_name/attachment_offset and standardized attachment_site.
- sim.env: more robust joint-to-actuator mapping across slidercrank/tendon transmissions.
- sim.env: revise get_scene_info() to return pose info and cache per step.
- MuJoCo viewer: hide left/right panels when launching the window.
- MuJoCo grippers: fix open/close handling for multi-actuator grippers.
- Internal: rename robits.sim.utils to robits.sim.mjcf_utils and refactor usages.
- Update CI workflow and drop Python 3.9 from version matrix 

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


[Unreleased]: https://github.com/markusgrotz/robits/compare/v0.8.2...HEAD
[0.8.2]: https://github.com/markusgrotz/robits/compare/v0.8.1...v0.8.2
[0.8.1]: https://github.com/markusgrotz/robits/compare/v0.8.0...v0.8.1
[0.8.0]: https://github.com/markusgrotz/robits/compare/v0.7.2...v0.8.0
[0.7.2]: https://github.com/markusgrotz/robits/compare/v0.7.1...v0.7.2
[0.7.1]: https://github.com/markusgrotz/robits/compare/v0.7.0...v0.7.1
[0.7.0]: https://github.com/markusgrotz/robits/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/markusgrotz/robits/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/markusgrotz/robits/compare/v0.5.2...v0.6.0
[0.5.2]: https://github.com/markusgrotz/robits/compare/v0.5.1...v0.5.2
[0.5.1]: https://github.com/markusgrotz/robits/compare/v0.5.0...v0.5.1
