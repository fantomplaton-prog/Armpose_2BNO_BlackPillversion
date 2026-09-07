# -*- coding: utf-8 -*-
"""
Вычисление калибровки по трём опорным позам (G/F/L).

ПОЧЕМУ ОТДЕЛЬНЫЙ МОДУЛЬ:
compute_calibration() принимает на вход только кватернионы (собранные
заранее, неважно откуда - хоть с BLE, хоть из файла с записанными
логами для тестов) и возвращает числа. Он НЕ знает про существование
BLE, потоков, input() и т.д. Такое разделение называют "чистое ядро,
грязные края" (functional core, imperative shell): вся логика с
сайд-эффектами (сеть, консоль, время) - в tracker.py и в примере
examples/cli_demo.py, а здесь - только вычисления, которые легко
проверить юнит-тестом (дать заранее известные кватернионы -> свериться
с ожидаемым результатом), не поднимая ни одного реального датчика.

Смысл самих поз (коротко, подробнее - в README/докстринге пакета):
  G - стоя прямо, руки вдоль тела, датчики неподвижны.
      -> ось z1 (в локальной системе груди) = мировая вертикаль.
      -> ось "вдоль руки" P (в локальной системе датчика руки) =
         направление мировой вертикали вниз (рука висит).
  F - рука вытянута вперёд (горизонтально), корпус не двигается.
      -> рассогласование по курсу (yaw) между "миром" датчика руки и
         "миром" датчика груди.
  L - корпус наклоняется вперёд из положения стоя.
      -> ось поворота этого наклона (в локальной системе груди) = x1
         (вправо); y1 достаётся из условия правой тройки x1 x y1 = z1.
"""

import math
from dataclasses import dataclass

import numpy as np

from .quat import (
    Quat,
    axis_from_quat,
    quat_conj,
    quat_from_matrix,
    quat_mult,
    quat_yaw,
    rotate_vector,
)

# Знак оси x1 (вправо), извлекаемой из наклона (калибровка L). Направление
# поворота при наклоне вперёд теоретически выведено, но если на реальном
# железе результат окажется зеркальным (право/лево или знак наклона в
# выводе перепутаны) - поменяйте это значение на противоположное при
# создании PoseTracker(sign_lean_axis=...). На нутацию/прецессию "вперёд"
# это не влияет, только на знак "вбок" и знак мирового наклона.
DEFAULT_SIGN_LEAN_AXIS = -1.0


@dataclass(frozen=True)
class Calibration:
    """Результат compute_calibration().

    ПОЧЕМУ dataclass, А НЕ ПРОСТО СЛОВАРЬ (как было в исходном скрипте,
    cal["q_body2sensor"]):
    У словаря произвольные строковые ключи - опечатка вроде
    cal["q_bodytosensor"] упадёт только в рантайме, при первом реальном
    обращении, и IDE тут не поможет с автодополнением. У dataclass поля
    зафиксированы в объявлении класса: cal.q_body2sensor - IDE
    подсказывает список полей, опечатка подсвечивается сразу при
    редактировании, а не через полчаса работы программы. Это отражает ту
    же цель, что и .h-файл в C: явно зафиксированный контракт "что здесь
    вообще есть". frozen=True делает объект неизменяемым после создания
    (как const в C) - калибровку нельзя случайно подменить одним полем
    где-то в середине программы.
    """

    # Обычные 3-элементные векторы направлений - тип np.ndarray (не Quat,
    # это не кватернионы, в них по 3 числа, а не 4).
    z1_local: np.ndarray        # мировая вертикаль в локальной системе груди
    x1_local: np.ndarray        # ось наклона (вправо) в локальной системе груди
    y1_local: np.ndarray        # достраивается по правой тройке x1 x y1 = z1
    arm_axis_local: np.ndarray  # направление "вдоль руки" в локальной системе датчика руки
    # А это настоящие кватернионы (4 числа w,x,y,z) - тип Quat, чтобы в
    # сигнатуре сразу было видно разницу с полями выше.
    q_body2sensor: Quat  # кватернион: система ТЕЛО -> локальная система груди
    q_yaw_align: Quat    # кватернион: "мир" датчика руки -> "мир" датчика груди
    delta_psi_deg: float  # то же рассогласование курса, в градусах (для вывода/лога)


def compute_calibration(q_c_g: Quat, q_a_g: Quat, q_c_f: Quat, q_a_f: Quat, q_c_l: Quat,
                         sign_lean_axis: float = DEFAULT_SIGN_LEAN_AXIS) -> Calibration:
    """Считает калибровку по кватернионам, снятым в трёх позах (G, F, L).

    Аргументы q_c_* / q_a_* - средние кватернионы груди/руки за время
    удержания соответствующей позы (обычно - результат
    BleQuatLink.sample_window(), см. ble_link.py).

    Бросает RuntimeError, если поза L была слишком мала для того, чтобы
    надёжно определить ось наклона (это единственная "физически" возможная
    ошибка калибровки - остальное просто линейная алгебра над тем, что
    дали).
    """
    # --- z1: мировая вертикаль в локальной системе груди (поза G) ---
    z1_local = rotate_vector(quat_conj(q_c_g), np.array([0.0, 0.0, 1.0]))
    z1_local = z1_local / np.linalg.norm(z1_local)

    # --- x1: ось наклона (G -> L), локальная система груди ---
    q_delta_lean = quat_mult(quat_conj(q_c_g), q_c_l)
    x1_raw = axis_from_quat(q_delta_lean) * sign_lean_axis
    x1_local = x1_raw - np.dot(x1_raw, z1_local) * z1_local
    x1_norm = np.linalg.norm(x1_local)
    if x1_norm < 1e-4:
        raise RuntimeError("Калибровка L: не удалось определить ось наклона "
                            "(наклон был слишком мал или отсутствовал)")
    x1_local = x1_local / x1_norm

    # --- y1 из условия правой тройки x1 x y1 = z1  =>  y1 = z1 x x1 ---
    y1_local = np.cross(z1_local, x1_local)
    y1_local = y1_local / np.linalg.norm(y1_local)

    r_cal = np.column_stack([x1_local, y1_local, z1_local])  # body -> chest-local
    q_body2sensor = quat_from_matrix(r_cal)

    # --- направление вдоль руки (G: рука висит вниз) ---
    arm_axis_local = rotate_vector(quat_conj(q_a_g), np.array([0.0, 0.0, -1.0]))
    arm_axis_local = arm_axis_local / np.linalg.norm(arm_axis_local)

    # --- согласование "мира" руки и "мира" груди по F (рука вытянута вперёд) ---
    q_body2world_f = quat_mult(q_c_f, q_body2sensor)
    y1_world_f = rotate_vector(q_body2world_f, np.array([0.0, 1.0, 0.0]))
    pointing_world_arm_f = rotate_vector(q_a_f, arm_axis_local)

    angle_body = math.atan2(y1_world_f[1], y1_world_f[0])
    angle_arm = math.atan2(pointing_world_arm_f[1], pointing_world_arm_f[0])
    delta_psi = angle_body - angle_arm
    q_yaw_align = quat_yaw(delta_psi)

    return Calibration(
        z1_local=z1_local,
        x1_local=x1_local,
        y1_local=y1_local,
        q_body2sensor=q_body2sensor,
        arm_axis_local=arm_axis_local,
        q_yaw_align=q_yaw_align,
        delta_psi_deg=math.degrees(delta_psi),
    )
