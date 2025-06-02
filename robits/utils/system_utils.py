import logging
import subprocess

logger = logging.getLogger(__name__)


def has_rt_support() -> bool:
    """
    Checks if the system has real-time support

    :returns: True if a RT kernel is installed
    """
    import subprocess

    result = subprocess.run(["uname", "-r"], check=True, capture_output=True, text=True)
    return "rt" in result.stdout


def ping_host(hostname) -> bool:
    """
    Checks if a host is available.

    :param hostname: the hostname or ip address to ping
    :returns: True if the ping command was successful
    """
    try:
        command = ["ping", "-c", "4", hostname]
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        _stdout, stderr = process.communicate()
        if process.returncode == 0:
            logger.info(f"Ping to {hostname} successful:")
            return True
        logger.warning(f"Ping to {hostname} failed: %s", stderr.decode())
        return False
    except FileNotFoundError:
        logger.error("Error: ping command not found.")
    except Exception as e:
        logger.error("Unable to run ping. Exception is: %s", e)
    return False
