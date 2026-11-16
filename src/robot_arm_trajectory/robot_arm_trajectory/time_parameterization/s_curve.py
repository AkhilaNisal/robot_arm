"""Deprecated import alias for the seventh-order polynomial profile."""

import warnings

from .seventh_order import SeventhOrderProfile


class SCurveProfile(SeventhOrderProfile):
    """Deprecated alias; this is not a conventional seven-phase S-curve."""

    def generate(self, *args, **kwargs):
        warnings.warn(
            "SCurveProfile is deprecated; use SeventhOrderProfile or the "
            "'seventh_order' method. This is a polynomial profile, not a "
            "seven-phase constant-jerk S-curve.",
            DeprecationWarning,
            stacklevel=2,
        )
        return super().generate(*args, **kwargs)
