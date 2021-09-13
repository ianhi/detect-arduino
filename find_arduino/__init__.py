try:
    from ._version import __version__
except:
    __version__ = "unknown"
from .detection import check_ports

__all__ = [
    "__version__",
    "check_ports",
]
