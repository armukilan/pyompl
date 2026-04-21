# from ._core import hello, RealVectorStateSpace
import sys
import platform

# Check Python version
if sys.version_info < (3, 10) or sys.version_info >= (3, 14):
    raise RuntimeError(
        "pyompl requires Python 3.10 to 3.13. "
        f"You are using Python {sys.version}. "
        "Please use a supported Python version."
    )

# Check Windows 32-bit
if sys.maxsize <= 2**32 and platform.system() == "Windows":
    raise RuntimeError(
        "pyompl does not support Windows 32-bit. "
        "Please use a 64-bit Python installation."
    )

# Check macOS version
if platform.system() == "Darwin":
    mac_version = tuple(int(x) for x in platform.mac_ver()[0].split(".")[:2])
    if mac_version < (15, 0):
        raise RuntimeError(
            "pyompl requires macOS 15.0 (Sequoia) or later. "
            f"You are using macOS {platform.mac_ver()[0]}. "
            "Please upgrade your macOS."
        )

# Check Linux glibc version
if platform.system() == "Linux":
    try:
        glibc_version = tuple(int(x) for x in platform.libc_ver()[1].split(".")[:2])
        if glibc_version < (2, 34):
            raise RuntimeError(
                "pyompl requires glibc 2.34 or later. "
                "Supported: Ubuntu 22.04+, Debian 12+, RHEL 9+. "
                f"Your system has glibc {platform.libc_ver()[1]}."
            )
    except Exception:
        pass

# Import compiled module
# try:
#     from ._core import hello, RealVectorStateSpace
# except ImportError as e:
#     raise ImportError(
#         "pyompl could not load the compiled OMPL module. "
#         "Your platform may not be supported. "
#         "Please check https://github.com/armukilan/pyompl "
#         f"for supported platforms. Original error: {e}"
#     )



from ._core import (
    SanityChecks,
    StateSpace,
    CompoundStateSpace,
    RealVectorBounds,
    RealVectorState,
    RealVectorStateSampler,
    RealVectorStateSpace,
    SO2State,
    SO2StateSampler,
    SO2StateSpace,
    SO3State, SO3StateSampler, SO3StateSpace
)

__all__ = [
    "SanityChecks",
    "StateSpace",
    "CompoundStateSpace",
    "RealVectorBounds",
    "RealVectorState",
    "RealVectorStateSpace",
]