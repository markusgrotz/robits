from typing import Tuple
from typing import List
from typing import Optional

import logging
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation as R

import mujoco

from robits.core.abc.control import ControllerBase
from robits.core.abc.control import ControlManager
from robits.core.abc.control import control_types

from robits.sim.env_client import MujocoJointControlClient
from robits.core.utils import FrequencyTimer

from robits.utils.transform_utils import transform_pose


logger = logging.getLogger(__name__)


class MujocoPositionControl(ControllerBase, MujocoJointControlClient):
    """
    Position control for Mujoco
    """

    def __init__(self, joint_names: List[str], actuator_names: List[str]) -> None:
        ControllerBase.__init__(self, control_types.position)
        MujocoJointControlClient.__init__(self, joint_names, actuator_names)

        self._target_lock = threading.Lock()
        self.target_positions: Optional[np.ndarray] = None
        self.control_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.wait_condition = threading.Condition()
        self.frequency_timer = FrequencyTimer(100)

    def start_controller(self) -> None:
        if self.control_thread is None or not self.control_thread.is_alive():
            self.stop_event.clear()
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.start()

    def stop_controller(self) -> None:
        if self.control_thread and self.control_thread.is_alive():
            self.stop_event.set()
            self.control_thread.join()
        self.target_positions = None

    def update(self, joint_positions: np.ndarray, relative: bool = False) -> None:
        if relative:
            current_joint_positions = self.get_current_joint_positions()
            joint_positions += current_joint_positions

        with self._target_lock:
            self.target_positions = joint_positions

        if not self.asynchronous:
            self._wait()

    def _wait(self):
        with self.wait_condition:
            while not self.stop_event.is_set():
                if self.wait_condition.wait(timeout=0.1):
                    break

    def _control_loop(self):
        self.frequency_timer.reset()
        while not self.stop_event.is_set():
            if self.target_positions is not None:
                current_joint_positions = self.get_current_joint_positions()
                error = self.target_positions - current_joint_positions

                step = 0.4 * error
                step = np.clip(
                    step, -0.5 * np.ones_like(error), 0.5 * np.ones_like(error)
                )

                self.data.ctrl[self.actuator_ids] = current_joint_positions + step

                joint_pos_reached = np.linalg.norm(error) < 0.1
                is_stopped = np.linalg.norm(step) < 0.01

                if joint_pos_reached or is_stopped:
                    logger.info("Target position reached.")
                    self.target_positions = None

                    with self.wait_condition:
                        self.wait_condition.notify_all()

            self.frequency_timer.wait_for_cycle()
        logger.info("Control loop stopped.")


class MujocoCartesianControl(ControllerBase, MujocoJointControlClient):
    """
    Cartesian control for Mujoco
    """

    def __init__(
        self,
        joint_names: List[str],
        actuator_names: List[str],
        site,
        transform_robot_to_world: np.ndarray,
    ) -> None:
        ControllerBase.__init__(self, control_types.cartesian)
        MujocoJointControlClient.__init__(self, joint_names, actuator_names)

        self.site = site
        self.transform_robot_to_world = transform_robot_to_world

        self._target_lock = threading.Lock()
        self.target_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.control_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.wait_condition = threading.Condition()
        self.frequency_timer = FrequencyTimer(100)

    def start_controller(self) -> None:
        if self.control_thread is None or not self.control_thread.is_alive():
            self.stop_event.clear()
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.start()

    def stop_controller(self) -> None:
        if self.control_thread and self.control_thread.is_alive():
            self.stop_event.set()
            self.control_thread.join()
        with self._target_lock:
            self.target_pose = None

    def update(self, pose: Tuple[np.ndarray, np.ndarray], relative: bool = False):
        assert len(pose) == 2
        position, quaternion = pose

        if relative:
            site_id = self.site.id

            robot_position, robot_matrix = self.data.site(site_id).xpos, self.data.site(
                site_id
            ).xmat.reshape((3, 3))
            robot_quaternion = R.from_matrix(robot_matrix).as_quat()

            robot_position, robot_quaternion = transform_pose(
                np.linalg.inv(self.transform_robot_to_world),
                robot_position,
                robot_quaternion,
            )

            position = robot_position + position
            quaternion = (
                R.from_quat(robot_quaternion) * R.from_quat(quaternion)
            ).as_quat()

        # Transform to world
        position, quaternion = transform_pose(
            self.transform_robot_to_world, position, quaternion
        )

        with self._target_lock:
            self.target_pose = (position, quaternion)

        if not self.asynchronous:
            self._wait()

    def _wait(self):
        with self.wait_condition:
            while not self.stop_event.is_set():
                if self.wait_condition.wait(timeout=0.1):
                    break

    def _control_loop(self):
        model, data = self.model, self.data
        site_id = self.site.id

        while not self.stop_event.is_set():
            with self._target_lock:
                if self.target_pose is None:
                    time.sleep(0.01)
                    continue
                position, quaternion = self.target_pose
                self.target_pose = None

            damping = 1e-2
            integration_dt = 1.0
            max_angvel = 0.2
            jac = np.zeros((6, model.nv))
            diag = damping * np.eye(6)
            error = np.zeros(6)
            error_pos = error[:3]
            error_ori = error[3:]
            site_quat = np.zeros(4)
            site_quat_conj = np.zeros(4)
            error_quat = np.zeros(4)

            pos_threshold = 0.01
            ori_threshold = 0.03
            pos_achieved = False
            ori_achieved = False
            quaternion_wxyz = np.concatenate(
                [quaternion[3:], quaternion[:3]], axis=None
            )

            self.frequency_timer.reset()
            while (
                not pos_achieved or not ori_achieved
            ) and not self.stop_event.is_set():
                with self._target_lock:
                    if self.target_pose is not None:
                        logger.debug(
                            "New target received. Interrupting current control."
                        )
                        break

                error_pos[:] = position - data.site(site_id).xpos
                mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
                mujoco.mju_negQuat(site_quat_conj, site_quat)
                mujoco.mju_mulQuat(error_quat, quaternion_wxyz, site_quat_conj)
                mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)

                mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
                dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, error)

                if max_angvel > 0:
                    dq_abs_max = np.abs(dq).max()
                    if dq_abs_max > max_angvel:
                        dq *= max_angvel / dq_abs_max

                q = data.qpos.copy()
                mujoco.mj_integratePos(model, q, dq, integration_dt)
                q = q[self.env.num_free_joints * 6 :]
                np.clip(q, *model.jnt_range.T, out=q)
                data.ctrl[self.actuator_ids] = q[self.joint_ids]

                pos_achieved = np.linalg.norm(error_pos) <= pos_threshold
                ori_achieved = np.linalg.norm(error_ori) <= ori_threshold

                if pos_achieved and ori_achieved:
                    logger.info("Target reached.")
                    break

                self.frequency_timer.wait_for_cycle()

            logger.debug(
                "Control loop finished. Target reached or interrupted by new target."
            )
            with self.wait_condition:
                self.wait_condition.notify_all()


class MujocoControlManager(ControlManager):

    def __init__(
        self,
        joint_names,
        actuator_names,
        site,
        default_joint_positions,
        transform_robot_to_world,
    ) -> None:
        super().__init__()

        self.register_controller(MujocoPositionControl(joint_names, actuator_names))
        self.register_controller(
            MujocoCartesianControl(
                joint_names, actuator_names, site, transform_robot_to_world
            )
        )
        # from robits.mujoco.control_mink import MinkCartesianWayointController
        # self.register_controller(MinkCartesianWayointController(env, self.controllers["position"], ee_pose_fn))

        self.default_joint_positions = default_joint_positions

    def move_home(self) -> None:
        with self(control_types.position) as ctrl:
            ctrl.update(self.default_joint_positions)

    def stop(self) -> None:
        logger.info("Shutting down control manager.")
