# -*- coding: utf-8 -*-
"""
Из калибровки (Calibration) и "сырых" кватернионов груди/руки в текущий
момент - в углы (нутация/прецессия/наклон корпуса), а из углов - в номера
зон по сетке, которую задаёт пользователь в начале сеанса.

Опять же: только вычисления, ни одной операции ввода-вывода. compute_outputs
и classify_pose_grid можно вызвать хоть в юнит-тесте с руками заданными
числами, хоть внутри PoseTracker.get_pose() с реальными данными с BLE -
для этого кода разницы нет.
"""

import math
from dataclasses import dataclass

import numpy as np

from .calibration import Calibration
from .quat import Quat, quat_conj, quat_mult, rotate_vector


def compute_outputs(q_chest: Quat, q_arm: Quat, cal: Calibration) -> tuple[float, float, float, float]:
    """Возвращает (нутация_град, прецессия_град, наклон_xz_град, наклон_yz_град).

    Нутация - угол руки от вертикали z1 системы ТЕЛО: 0° - рука вверх,
    180° - рука вниз, 90° - горизонтально. Физически это угол между двумя
    векторами (arccos), поэтому диапазон всегда [0°, 180°] - "нутация
    270°" в принципе не бывает, это не полный круг, а как широта на
    глобусе (0..180, полюс-полюс), в отличие от прецессии ниже.

    Прецессия - угол в плоскости x1y1 системы ТЕЛО: 0° - вперёд,
    +90° - вправо, -90° - влево, ±180° - назад. Это atan2, то есть уже
    полный круг (-180°, 180°], как долгота на глобусе.

    Наклон корпуса XZ/YZ - в МИРОВОЙ системе (не системе тела) - отдельная
    величина, не завязана на положение руки.
    """
    q_body2world = quat_mult(q_chest, cal.q_body2sensor)
    q_arm2world = quat_mult(cal.q_yaw_align, q_arm)
    q_arm2body = quat_mult(quat_conj(q_body2world), q_arm2world)

    pointing_body = rotate_vector(q_arm2body, cal.arm_axis_local)
    bz = float(np.clip(pointing_body[2], -1.0, 1.0))
    nutation_deg = math.degrees(math.acos(bz))
    precession_deg = math.degrees(math.atan2(pointing_body[0], pointing_body[1]))

    z1_world = rotate_vector(q_chest, cal.z1_local)
    tilt_xz_deg = math.degrees(math.atan2(z1_world[0], z1_world[2]))
    tilt_yz_deg = math.degrees(math.atan2(z1_world[1], z1_world[2]))

    return nutation_deg, precession_deg, tilt_xz_deg, tilt_yz_deg


def classify_pose_grid(nutation_deg: float, precession_deg: float,
                        n_nutation_zones: int, n_precession_zones: int) -> tuple[int, int]:
    """Возвращает (nutation_zone, precession_zone) - номера зон, считая с 1.

    Нутация: диапазон [0°, 180°] делится на n_nutation_zones равных зон
    край в край (зона 1 = [0, 180/N), ...). "Центрировать" тут нечего:
    0° и 180° - это физические полюса (строго вверх / строго вниз), а не
    условная точка отсчёта на окружности.

    Прецессия: полный круг (360°) делится на n_precession_zones равных
    зон КРАЙ В КРАЙ, начиная РОВНО от 0° ("вперёд") и дальше по часовой
    стрелке (в сторону +90°/"вправо" - см. соглашение о знаках в
    compute_outputs выше): зона 1 = [0°, 360/M), зона 2 = [360/M, 2*360/M)
    и т.д. Раньше зона 1 была центрирована на 0° (границы [-180/M,+180/M)),
    но при малом числе зон это могло свести "рука влево" (-45°) и "рука
    вправо" (+45°) в одну и ту же широкую зону, если она охватывала пол-
    оборота вокруг 0° сразу в обе стороны - а это разные, физически
    отличимые положения руки, которые обязаны попадать в разные зоны.
    """
    nutation_deg = max(0.0, min(180.0, nutation_deg))
    nut_width = 180.0 / n_nutation_zones
    nutation_zone = min(int(nutation_deg / nut_width) + 1, n_nutation_zones)

    prec_width = 360.0 / n_precession_zones
    shifted = precession_deg % 360.0
    precession_zone = min(int(shifted / prec_width) + 1, n_precession_zones)

    return nutation_zone, precession_zone


@dataclass(frozen=True)
class PoseResult:
    """Один "снимок" положения руки - то, что возвращает PoseTracker.get_pose().

    ПОЧЕМУ dataclass, А НЕ ПРОСТО КОРТЕЖ (nutation, precession, ...):
    С кортежем потребитель вынужден помнить порядок полей и распаковывать
    его вручную: nutation, precession, tilt_xz, tilt_yz, nz, pz = tracker.get_pose()
    - перепутать местами nutation_zone/precession_zone легко, а компилятора,
    который бы это поймал, в Python нет. С именованными полями это
    read.nutation_zone - опечатка в имени поля - это AttributeError сразу,
    а не тихо неправильные данные через два поля. Для "чёрного ящика",
    которым будет пользоваться кто-то другой (или ты сам через полгода),
    это особенно важно - самодокументируемый результат вместо "помни
    порядок из документации".
    """

    nutation_deg: float
    precession_deg: float
    tilt_xz_deg: float
    tilt_yz_deg: float
    nutation_zone: int
    precession_zone: int
    n_nutation_zones: int
    n_precession_zones: int
