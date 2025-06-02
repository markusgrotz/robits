from typing import Dict
from typing import Any
from typing import Optional

import logging
import threading

from enum import unique
from robits.core.compat import StrEnum

logger = logging.getLogger(__name__)


@unique
class ControlTypes(StrEnum):
    """
    Class representing different types of control methods.
    """

    position = "position"
    cartesian = "cartesian"
    motion_planning = "motion_planning"


control_types = ControlTypes


class ControllerBase:
    """
    Base class for control methods.
    """

    def __init__(
        self, controller_type: ControlTypes, asynchronous: bool = False
    ) -> None:
        """
        Initialize the controller.

        :param controller_type: Type of the controller.
        :param asynchronous: Flag if the execution is asynchronous or not
        """
        self.controller_type = controller_type
        self.asynchronous = asynchronous
        self.previous_config: Dict[str, Any] = {}

    def update(self, *args, **kwargs):
        """
        Abstract method to update the controller's command target.
        Concrete subclasses must implement this.
        """
        raise NotImplementedError("Not implemented yet.")

    def set_asynchronous(self) -> None:
        """
        Set the controller to asynchronous mode.
        """
        self.asynchronous = True

    def set_synchronous(self) -> None:
        """
        Set the controller to synchronous mode.
        """
        self.asynchronous = False

    def start_controller(self) -> None:
        """
        Start the controller.
        """
        logger.info("starting controller %s", self.controller_type)

    def stop_controller(self) -> None:
        """
        Stop the controller.
        """
        logger.info("stopping controller %s", self.controller_type)

    def configure(self, asynchronous: bool, **kwargs) -> None:
        """
        Configure the controller

        :param asynchronous: whether the controller should be in asynchronous mode
        """
        if asynchronous:
            self.set_asynchronous()
        else:
            self.set_synchronous()

    def reset_configuration(self) -> None:
        """
        Restores a previous configuration
        """
        self.configure(**self.previous_config)

    def save_current_config(self) -> None:
        """
        Save the current configuration
        """
        self.previous_config = {"asynchronous": self.asynchronous}

    def __enter__(self) -> "ControllerBase":
        """
        Start the controller when entering a context.

        :return: Self instance.
        """
        self.start_controller()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """
        Stop the controller when exiting a context.

        :param exc_type: Exception type.
        :param exc_value: Exception value.
        :param traceback: Traceback information.
        """
        self.stop_controller()


class ControlManager:
    """
    Manages multiple controllers and monitors their activation.
    Only one controller should be active at the same time
    """

    def __init__(self) -> None:
        """
        Initialize the control manager.
        """
        self.controllers: Dict[str, ControllerBase] = {}
        self.active_controller: Optional[ControllerBase] = None
        self.reset_controller_config = True

    def register_controller(self, controller: ControllerBase) -> None:
        """
        Register a controller.

        :param controller: Controller instance to register.
        """
        controller_type = controller.controller_type
        if controller_type in self.controllers:
            raise ValueError("Controller already registered.")

        self.controllers[controller_type] = controller

    def stop(self) -> None:
        """
        Stop the active controller.
        """
        if not self.active_controller:
            logger.info("No controller currently selected.")
            return

        self.active_controller.stop_controller()
        self.active_controller = None

    def __call__(
        self,
        controller_type: ControlTypes,
        reset_controller_config: bool = True,
        asynchronous: bool = False,
        **kwargs
    ) -> "ControlManager":
        """
        Activate a controller by type.

        :param controller_type: Type of the controller to activate.
        :param reset_controller_config: Flag whether to reset the configuration after execution is done
        :param asynchronous: Flag whether the controller should be in asynchronous mode
        :param kwargs: Additional parameters to configure the controller
        :return: Self instance
        """
        if controller_type not in self.controllers:
            raise ValueError("Invalid controller type. Controller is not registered.")

        if self.active_controller:
            logger.error("Robot is currently controlled. Please disable controllers.")
            return self

        self.active_controller = self.controllers[controller_type]

        self.reset_controller_config = reset_controller_config
        if reset_controller_config:
            self.active_controller.save_current_config()
        self.active_controller.configure(asynchronous, **kwargs)

        return self

    def __enter__(self) -> ControllerBase:
        """
        Enter the control manager's context.

        :return: Active controller instance.
        """
        if not self.active_controller:
            raise RuntimeError("No controller currently selected.")
        return self.active_controller.__enter__()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """
        Exit the control manager's context, stopping controllers.

        :param exc_type: Exception type.
        :param exc_value: Exception value.
        :param traceback: Traceback information.
        """
        if not self.active_controller:
            logger.error("No controller currently selected.")
            return
        try:
            self.active_controller.__exit__(exc_type, exc_value, traceback)
        except Exception as e:
            logger.error("Unable to stop controllers! Exception was: %s", e)
            raise e
        finally:
            if self.reset_controller_config:
                self.active_controller.reset_configuration()
            self.active_controller = None

    def move_home(self) -> None:
        """
        Convenience function to move the robot to default joint position
        """
        pass


class BimanualControlManager(ControlManager):
    """
    Manages two control managers for bimanual control.
    The control is delegated to the both arms by spawning a thread.

    .. todo:: Can be implemented more efficiently by reusing the thread.
    """

    def __init__(
        self, control_right: ControlManager, control_left: ControlManager
    ) -> None:
        """
        Initialize the bimanual control manager.

        :param control_right: Control manager for the right side.
        :param control_left: Control manager for the left side.
        """
        super().__init__()
        self.control_right = control_right
        self.control_left = control_left

    def __call__(self, *args, **kwargs) -> "BimanualControlManager":

        self.control_right(*args, **kwargs)
        self.control_left(*args, **kwargs)

        return self

    def __enter__(self):

        right_ctrl = self.control_right.__enter__()
        left_ctrl = self.control_left.__enter__()

        class ControlDelegator(ControllerBase):
            def update(self, right, left, **kwargs):

                right_thread = threading.Thread(
                    target=right_ctrl.update, args=(right,), kwargs=kwargs
                )
                left_thread = threading.Thread(
                    target=left_ctrl.update, args=(left,), kwargs=kwargs
                )

                right_thread.start()
                left_thread.start()

                if not right_ctrl.asynchronous:

                    right_thread.join()
                    left_thread.join()

        return ControlDelegator(right_ctrl.controller_type)

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.control_right.__exit__(exc_type, exc_value, traceback)
        self.control_left.__exit__(exc_type, exc_value, traceback)

    def move_home(self) -> None:
        right_thread = threading.Thread(target=self.control_right.move_home)
        left_thread = threading.Thread(target=self.control_left.move_home)

        right_thread.start()
        left_thread.start()

        right_thread.join()
        left_thread.join()
