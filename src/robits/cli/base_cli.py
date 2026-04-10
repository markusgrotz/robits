"""
Base module for RoBits command-line interface.

This module provides the foundation for all CLI commands in RoBits.
It sets up the base command group, configures logging with Rich formatting,
and provides global verbosity controls.
"""

import sys
import logging

import rich_click as click

from rich.logging import RichHandler
from rich.console import Console

from robits.core import __version__


logger = logging.getLogger(__name__)

console = Console()


@click.group()
@click.option(
    "--verbose",
    "-v",
    default=False,
    count=True,
    help="Increase verbosity of the logging output. Can be used multiple times.",
)
@click.option(
    "--quiet",
    "-q",
    default=False,
    is_flag=True,
    help="Suppress all logging output except critical and error messages",
)
def cli(verbose: int, quiet: bool) -> None:
    """
    RoBits command line interface for robotic manipulation.

    This is the main entry point for all RoBits CLI commands. It provides
    global verbosity controls and sets up logging configuration.

    Use --verbose/-v to increase log detail level (can be used multiple times).
    Use --quiet/-q to suppress all but error messages.

    Example: rb -vv move home --robot-name robot_panda_sim

    See https://robits.readthedocs.org for more documentation and setup.
    """
    log_config = {
        "handlers": [RichHandler(console=console, markup=True, rich_tracebacks=True)]
    }
    if verbose and quiet:
        logger.error("verbose and quiet must be mutually exclusive")
        sys.exit(-1)
    elif verbose == 1:
        logging.basicConfig(level=logging.INFO, **log_config)
    elif verbose == 2:
        logging.basicConfig(level=logging.DEBUG, **log_config)
    elif verbose >= 3:
        logging.basicConfig(level=logging.NOTSET, **log_config)
    elif quiet:
        logging.basicConfig(level=logging.ERROR, **log_config)
    else:
        logging.basicConfig(level=logging.WARNING, **log_config)

    import numpy as np

    np.set_printoptions(precision=4, suppress=True)

    logger.debug("Current version is %s", __version__)
