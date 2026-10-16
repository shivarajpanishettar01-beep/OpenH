"""Image metrics with explicit definitions. None means "cannot be computed", never zero.

These are display aids for the demo. None of them is a validated success criterion.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

Roi = Tuple[int, int, int, int]      # row0, row1, col0, col1 (pixels)


def _crop(img, roi: Roi):
    r0, r1, c0, c1 = roi
    a = img[max(r0, 0):r1, max(c0, 0):c1]
    return a.astype(np.float32) if a.size >= 16 else None


def speckle_snr(img: Optional[np.ndarray], roi: Roi) -> Optional[float]:
    """mean / std of the displayed B-mode intensity in a homogeneous region.

    On log-compressed images this is only a relative indicator (the ~1.91 Rayleigh value applies
    to the envelope before compression).
    """
    if img is None:
        return None
    a = _crop(img, roi)
    if a is None or a.std() < 1e-6:
        return None
    return float(a.mean() / a.std())


def cnr(img: Optional[np.ndarray], roi_target: Optional[Roi], roi_bg: Roi) -> Optional[float]:
    """|mean_bg - mean_target| / sqrt(var_bg + var_target). Needs a target ROI (e.g. vessel lumen)."""
    if img is None or roi_target is None:
        return None
    t, b = _crop(img, roi_target), _crop(img, roi_bg)
    if t is None or b is None:
        return None
    den = np.sqrt(t.var() + b.var())
    return float(abs(b.mean() - t.mean()) / den) if den > 1e-6 else None


def coupling_mean(img: Optional[np.ndarray]) -> Optional[float]:
    """Mean B-mode intensity, as qc_monitor computes it for the coupling check."""
    if img is None:
        return None
    return float(img.mean())
