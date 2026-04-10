"""
Module for compatibility
"""

import sys

# StrEnum is only available in Python 3.11
if sys.version_info >= (3, 11):
    from enum import StrEnum
else:
    from enum import Enum

    class StrEnum(str, Enum):
        pass
