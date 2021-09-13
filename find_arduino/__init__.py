try:
    from ._version import __version__
except:
    __version__ = "unknown"
from .detection import find_arduino

__all__ = [
    "__version__",
    "find_arduino",
]
