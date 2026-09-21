# -*- coding: utf-8 -*-
"""
pronation_supination - угол пронации/супинации предплечья по двум BNO08x
(плечо + предплечье), не зависящий от угла сгибания локтя.

Публичный API (см. tracker.py про типичный сценарий использования):
    from pronation_supination import PronSupTracker
    tracker = PronSupTracker()
    tracker.connect()
    tracker.calibrate_hang(); tracker.calibrate_forward(); tracker.calibrate_zero()
    tracker.finalize_calibration()
    angle = tracker.get_angle()

Как и в posetracker/__init__.py - остальные файлы пакета (calibration.py,
pose.py, tracker.py) не видны потребителю напрямую, реэкспорт ниже - явный
контракт "что здесь вообще есть".
"""

from .calibration import PronSupCalibration, calibrate_pronation_supination
from .pose import DEFAULT_SIGN_TWIST, compute_pronation_supination, swing_twist_angle_deg
from .tracker import (
    DEFAULT_DURATION_FORWARD,
    DEFAULT_DURATION_HANG,
    DEFAULT_DURATION_ZERO,
    PronSupTracker,
)

__all__ = [
    "PronSupTracker",
    "PronSupCalibration",
    "calibrate_pronation_supination",
    "compute_pronation_supination",
    "swing_twist_angle_deg",
    "DEFAULT_SIGN_TWIST",
    "DEFAULT_DURATION_HANG",
    "DEFAULT_DURATION_FORWARD",
    "DEFAULT_DURATION_ZERO",
]
