"""Package logger and display detection."""

import logging

import screeninfo

logger = logging.getLogger("mujoco_toolbox")


def display_available() -> bool:
    """Whether at least one monitor is connected."""
    try:
        return bool(screeninfo.get_monitors())
    except screeninfo.ScreenInfoError:
        logger.info("No display detected; interactive rendering is disabled.")
        return False
