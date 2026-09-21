# -*- coding: utf-8 -*-
"""
Из калибровки (PronSupCalibration) и текущих кватернионов плеча/предплечья -
в угол пронации/супинации (swing-twist разложение). Только вычисления, ни
одной операции ввода-вывода - см. тот же принцип в posetracker/pose.py.
"""

import math

import numpy as np

from posetracker.quat import Quat, quat_conj, quat_mult

from .calibration import PronSupCalibration

# Знак итогового угла - чистое соглашение (что считать "положительной"
# пронацией). Подбирается по месту на реальном железе, как SIGN_LEAN_AXIS
# в основном проекте - если знак на практике окажется противоположным
# ожидаемому, просто поменяйте на -1.0 при создании PronSupTracker(...).
DEFAULT_SIGN_TWIST = 1.0


def swing_twist_angle_deg(q_rel: Quat, axis_local: np.ndarray) -> float:
    """Угол (градусы) компоненты TWIST кватерниона q_rel вокруг оси
    axis_local (ось должна быть выражена в той же локальной системе, что
    и "исходная" сторона q_rel - здесь это локальная система предплечья,
    см. compute_pronation_supination). Компонента SWING (куда сама ось
    развёрнута) в ответ не попадает и на угол не влияет - в этом и весь
    смысл разложения.

    ПОЧЕМУ w ПРИНУДИТЕЛЬНО >= 0 (двойное накрытие кватернионов):
    q и -q описывают ОДИН и тот же поворот (буквально все 4 числа с
    обратным знаком - та же физическая ориентация). BNO08x не гарантирует
    непрерывность знака - в какой-то момент представитель может незаметно
    "перевернуться" на -q, хотя физически ничего не изменилось. Так как
    здесь угол УДВАИВАЕТСЯ (2*atan2(...)), переворот знака (w,v)->(-w,-v)
    сдвигает atan2 на 180 -> после удвоения результат скачет ровно на 360
    (например -85 <-> 275 - один и тот же угол, просто с другой стороны
    "обёртки" - реальный баг, пойманный на живых данных). Фиксируем знак
    (тот же приём, что и в axis_from_quat в quat.py) - тогда atan2(s,w) с
    w>=0 всегда лежит в (-90,90], после удвоения непрерывный диапазон
    (-180,180] без скачков."""
    w = float(q_rel[0])
    v = q_rel[1:4]
    if w < 0:
        w = -w
        v = -v
    s = float(np.dot(v, axis_local))
    return math.degrees(2.0 * math.atan2(s, w))


def compute_pronation_supination(
    q_upper: Quat, q_forearm: Quat, cal: PronSupCalibration,
    sign_twist: float = DEFAULT_SIGN_TWIST,
) -> float:
    """Текущий угол пронации/супинации (градусы). 0° - поза калибровки zero
    (локоть ~90°, ладонь вверх). Не зависит от угла сгибания локтя и от
    того, как ориентировано само плечо в пространстве."""
    q_forearm_corrected = quat_mult(cal.q_yaw_align_elbow, q_forearm)
    q_rel = quat_mult(quat_conj(q_upper), q_forearm_corrected)
    q_rel_zeroed = quat_mult(quat_conj(cal.q_zero), q_rel)

    return sign_twist * swing_twist_angle_deg(q_rel_zeroed, cal.forearm_axis_local)


# ============================================================================
# САМОПРОВЕРКА (синтетические кватернионы с заранее известным ответом, без
# реального железа). Запуск: `python -m pronation_supination.pose` из
# Python/, или просто `py -3 pronation_supination/pose.py`.
# ============================================================================

def _quat_axis_angle(axis: np.ndarray, angle_deg: float) -> Quat:
    """Кватернион поворота на angle_deg вокруг произвольной оси - нужен
    только для построения тестовых данных ниже."""
    axis = axis / np.linalg.norm(axis)
    half = math.radians(angle_deg) / 2.0
    return np.array([math.cos(half), *(math.sin(half) * axis)])


def _self_test():
    from .calibration import calibrate_pronation_supination

    # Договорённость для теста: у обоих датчиков в их "нулевой" (identity)
    # ориентации локальная ось -Z уже совпадает с мировой (0,0,-1) - то
    # есть поза hang у обоих ровно identity, никакой возни с реальным
    # креплением датчика тут не нужно, это чисто синтетика.
    identity = np.array([1.0, 0.0, 0.0, 0.0])

    # Поза forward: поворот на 90 вокруг мировой оси X переводит локальную
    # (0,0,-1) в мировую (0,1,0) ("вперёд" = +Y). У ОБОИХ датчиков одна и
    # та же ориентация корпуса -> рассогласования курса быть не должно.
    q_forward = _quat_axis_angle(np.array([1.0, 0.0, 0.0]), 90.0)

    cal = calibrate_pronation_supination(
        q_upper_hang=identity, q_forearm_hang=identity,
        q_upper_forward=q_forward, q_forearm_forward=q_forward,
        q_upper_zero=q_forward, q_forearm_zero=q_forward,
    )

    # Тест 1: без рассогласования курса delta_psi должен быть ~0.
    yaw_check = 2.0 * math.degrees(math.atan2(cal.q_yaw_align_elbow[3], cal.q_yaw_align_elbow[0]))
    assert abs(yaw_check) < 1e-6, f"ожидали 0 рассогласования курса, получили {yaw_check}"

    # Тест 2: сама поза zero (без изменений) должна давать угол ровно 0.
    angle_at_zero = compute_pronation_supination(q_forward, q_forward, cal)
    assert abs(angle_at_zero) < 1e-6, f"в позе калибровки ожидали 0°, получили {angle_at_zero}"

    # Тест 3: поворачиваем ТОЛЬКО предплечье на +30 вокруг его собственной
    # продольной оси (cal.forearm_axis_local) относительно позы zero -
    # должны получить угол +30, независимо от того, что плечо не менялось.
    q_twist_30 = _quat_axis_angle(cal.forearm_axis_local, 30.0)
    q_forearm_twisted = quat_mult(q_forward, q_twist_30)
    angle_30 = compute_pronation_supination(q_forward, q_forearm_twisted, cal)
    assert abs(angle_30 - 30.0) < 1e-6, f"ожидали +30°, получили {angle_30}"

    # Тест 4 (ключевой - ради него всё и затевалось): та же скрутка +30,
    # но теперь ПЛЕЧО в момент измерения смотрит совсем в другую сторону
    # (согнули локоть на 90, подняли/повернули плечо) - угол пронации не
    # должен от этого измениться, раз реальная скрутка предплечья та же.
    q_upper_elsewhere = _quat_axis_angle(np.array([0.3, 0.8, 0.5]), 77.0)
    # То же самое q_rel (плечо->предплечье), что и в тесте 3, просто
    # "приставленное" к другой ориентации плеча: q_forearm = q_upper_new * q_rel_test3.
    q_rel_test3 = quat_mult(quat_conj(q_forward), q_forearm_twisted)
    q_forearm_elsewhere = quat_mult(q_upper_elsewhere, q_rel_test3)
    angle_30_elsewhere = compute_pronation_supination(q_upper_elsewhere, q_forearm_elsewhere, cal)
    assert abs(angle_30_elsewhere - 30.0) < 1e-6, (
        f"угол пронации не должен зависеть от ориентации плеча, "
        f"ожидали +30°, получили {angle_30_elsewhere}")

    # Тест 5 (регрессия на баг с двойным накрытием: -85 внезапно превращался
    # в 275 в одной четверти прецессии). Кватернион q_rel_zeroed и его
    # "перевёрнутый по знаку" двойник (-q, тот же физический поворот)
    # обязаны давать РОВНО одинаковый угол.
    q_minus30 = _quat_axis_angle(cal.forearm_axis_local, -30.0)
    q_forearm_minus30 = quat_mult(q_forward, q_minus30)

    angle_normal = compute_pronation_supination(q_forward, q_forearm_minus30, cal)
    angle_flipped = compute_pronation_supination(-q_forward, -q_forearm_minus30, cal)
    assert abs(angle_normal - (-30.0)) < 1e-6, f"ожидали -30°, получили {angle_normal}"
    assert abs(angle_flipped - angle_normal) < 1e-6, (
        f"баг с двойным накрытием вернулся: обычный кватернион дал {angle_normal}°, "
        f"перевёрнутый по знаку (тот же поворот!) дал {angle_flipped}°")

    print("Все самопроверки прошли: калибровка и swing-twist разложение работают корректно.")


if __name__ == "__main__":
    _self_test()
