# -*- coding: utf-8 -*-
"""
Калибровка пронации/супинации по трём опорным позам (hang/forward/zero).

ПОЧЕМУ ОТДЕЛЬНЫЙ МОДУЛЬ (как calibration.py в posetracker): только
вычисления по уже готовым кватернионам, ни одной операции ввода-вывода -
можно проверить юнит-тестом с руками заданными числами (см. самопроверку
в pose.py), не поднимая ни BLE, ни реальные датчики.

Смысл поз - подробнее в докстринге исходного pronation_supination.py
(перенесён сюда без изменений математики):
  hang    - рука прямая (локоть разогнут), висит вдоль тела.
      -> продольные оси плеча/предплечья в их локальных системах - через
         гравитацию, каждая независимо от другой.
  forward - рука прямая (локоть разогнут), вытянута вперёд горизонтально.
      -> рассогласование курса (yaw) между датчиком плеча и датчиком
         предплечья (у них независимые произвольные нули курса).
  zero    - локоть согнут ~90°, ладонь строго вверх.
      -> анатомический "ноль" пронации/супинации - точка отсчёта для всех
         дальнейших измерений.
"""

import math
from dataclasses import dataclass

import numpy as np

from posetracker.quat import Quat, quat_conj, quat_mult, quat_yaw, rotate_vector


@dataclass(frozen=True)
class PronSupCalibration:
    """Результат calibrate_pronation_supination() - см. docstring dataclass
    Calibration в posetracker/calibration.py про то, почему dataclass, а не
    словарь (тот же самый аргумент, дословно применим и здесь)."""

    upper_arm_axis_local: np.ndarray  # продольная ось плеча в её локальной системе (поза hang)
    forearm_axis_local: np.ndarray    # продольная ось предплечья в её локальной системе (поза hang)
    q_yaw_align_elbow: Quat           # выравнивание курса плечо<->предплечье (поза forward)
    q_zero: Quat                      # относительный поворот в позе zero - точка отсчёта угла


def calibrate_pronation_supination(
    q_upper_hang: Quat, q_forearm_hang: Quat,
    q_upper_forward: Quat, q_forearm_forward: Quat,
    q_upper_zero: Quat, q_forearm_zero: Quat,
) -> PronSupCalibration:
    """Считает калибровку по кватернионам, снятым в трёх позах (см. docstring
    модуля). Аргументы - средние кватернионы плеча/предплечья за время
    удержания соответствующей позы (усреднение - забота вызывающего кода,
    см. PronSupTracker.calibrate_hang/forward/zero в tracker.py)."""
    # --- поза hang: продольные оси через гравитацию, каждая сама по себе ---
    upper_arm_axis_local = rotate_vector(quat_conj(q_upper_hang), np.array([0.0, 0.0, -1.0]))
    upper_arm_axis_local = upper_arm_axis_local / np.linalg.norm(upper_arm_axis_local)

    forearm_axis_local = rotate_vector(quat_conj(q_forearm_hang), np.array([0.0, 0.0, -1.0]))
    forearm_axis_local = forearm_axis_local / np.linalg.norm(forearm_axis_local)

    # --- поза forward: рассогласование курса между плечом и предплечьем ---
    # Оба сейчас смотрят в одну и ту же истинную сторону (рука прямая,
    # вытянута вперёд) - любая разница в том, что каждый датчик САМ о себе
    # думает "куда это" - и есть рассогласование курса между ними.
    pointing_world_upper = rotate_vector(q_upper_forward, upper_arm_axis_local)
    pointing_world_forearm = rotate_vector(q_forearm_forward, forearm_axis_local)

    angle_upper = math.atan2(pointing_world_upper[1], pointing_world_upper[0])
    angle_forearm = math.atan2(pointing_world_forearm[1], pointing_world_forearm[0])
    delta_psi = angle_upper - angle_forearm
    q_yaw_align_elbow = quat_yaw(delta_psi)

    # --- поза zero: относительная ориентация в этот момент = точка отсчёта ---
    q_forearm_zero_corrected = quat_mult(q_yaw_align_elbow, q_forearm_zero)
    q_zero = quat_mult(quat_conj(q_upper_zero), q_forearm_zero_corrected)

    return PronSupCalibration(
        upper_arm_axis_local=upper_arm_axis_local,
        forearm_axis_local=forearm_axis_local,
        q_yaw_align_elbow=q_yaw_align_elbow,
        q_zero=q_zero,
    )
