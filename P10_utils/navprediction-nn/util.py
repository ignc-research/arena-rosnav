import logging
import sys
from functools import wraps
from time import time
from rich.logging import RichHandler


def new_logger(module_name: str = None, level: str = "INFO", stream=sys.stdout) -> logging.Logger:
    """Initialize logger with level and return logger"""

    if module_name is None:
        module_name = f"{__name__}".replace("__", "")

    levels = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL,
    }

    logging.basicConfig(
        level=levels[level],
        format=f"{module_name}: %(message)s",
        datefmt="[%X]",
        handlers=[RichHandler()],
    )

    return logging.getLogger(module_name)


def timeit(f):
    @wraps(f)
    def wrap(*args, **kw):
        ts = time()
        result = f(*args, **kw)
        te = time()
        new_logger().info("Func: %s() took: %2.4f sec" % (f.__name__, te - ts))
        return result

    return wrap
