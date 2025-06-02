from typing import List
from typing import Dict
from typing import Any

import psutil
from functools import partial
import time


from robits.utils.service_launcher import MAGIC_CMD_ARG


class ProcessMonitor:
    """
    Monitors running service instances. Running services are detected by a
    magic command line parameter.
    """

    def list(self) -> List[str]:
        running_services: List[str] = []
        for process_info in self.status():
            running_services.append(process_info["cmd"][-1])
        return running_services

    def status(self) -> List[Dict[str, Any]]:
        status: List[Dict[str, Any]] = []
        magic_string = f"--magic-string {MAGIC_CMD_ARG}"
        fun = partial(ProcessMonitor.filter_by_cmdline, magic_string)

        for p in filter(fun, psutil.process_iter()):
            process_info = {
                "status": p.status(),
                "name": p.name(),
                "pid": p.pid,
                "exec": p.exe(),
                "cmd": p.cmdline(),
            }
            status.append(process_info)
        return status

    def terminate(self, config_name: str):
        terminated = []
        magic_string = f"--magic-string {MAGIC_CMD_ARG} {config_name}"
        fun = partial(ProcessMonitor.filter_by_cmdline, magic_string)
        for p in filter(fun, psutil.process_iter()):
            p.terminate()
            terminated.append(f"{p.pid} - {p.name()}")

        return terminated

    def kill(self, config_name: str):
        terminated = []
        magic_string = f"--magic-string {MAGIC_CMD_ARG} {config_name}"
        fun = partial(ProcessMonitor.filter_by_cmdline, magic_string)
        running = False
        for p in filter(fun, psutil.process_iter()):
            if p.is_running():
                running = True

        if running:
            time.sleep(3)

        for p in filter(fun, psutil.process_iter()):
            p.kill()
            terminated.append(f"{p.pid} - {p.name()}")

        return terminated

    @classmethod
    def filter_by_cmdline(cls, arg, p):
        try:
            if "python" in p.exe():
                return arg in " ".join(p.cmdline())
        except psutil.AccessDenied:
            pass
        return False
