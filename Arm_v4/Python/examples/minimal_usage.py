# -*- coding: utf-8 -*-
"""
МИНИМАЛЬНЫЙ пример использования пакета posetracker "как чёрного ящика" -
без красивого обратного отсчёта, без перекалибровки по команде 'r', без
фонового чтения stdin. Именно ТАКОЙ код и написал бы коллега в СВОЁМ
проекте после `pip install -e <путь до Python/>` (см. предыдущий разговор
про то, как это доставить на другую машину) - ему не нужно знать вообще
ничего про BLE/SPI/STM32/кватернионы, только 4 метода PoseTracker.

Сравни с examples/cli_demo.py: там та же самая логика, только обвешана
удобствами для интерактивной консоли (countdown, перекалибровка по 'r',
проверка ошибок с понятными сообщениями). Здесь - голый минимум, чтобы
было видно сам "контракт" библиотеки без лишнего шума вокруг.
"""

import time

from posetracker import PoseTracker

# Сколько зон прецессии всего - вынесено в переменную (не просто число
# внутри PoseTracker(...) ниже), потому что то же число нужно ещё и для
# расчёта коэффициента a (см. linear_dip_coefficient) - если бы число зон
# было зашито в двух местах отдельно, при изменении легко забыть поправить
# оба. Взял 360 (как в примере: "зона" = "градус"), чтобы min_zone было
# удобно понимать сразу в градусах - но это не обязательно, работает с
# любым числом зон.
N_PRECESSION_ZONES = 360


def linear_dip_coefficient(zone: int, n_zones: int, min_zone: int,
                            min_coef: float, ramp_zones: int) -> float:
    """Коэффициент a: 1.0 (стандартный) везде, кроме "провала" вокруг
    zone=min_zone - там a=min_coef, и линейно возвращается к 1.0 на
    расстоянии ramp_zones зон в КАЖДУЮ сторону от min_zone.

    Расстояние считается ПО КРУГУ (зона n_zones соседствует с зоной 1) -
    это важно именно для прецессии, которая описывает полный оборот на
    360°, а не для нутации (там 0°/180° - настоящие физические полюса,
    а не точки "смыкания" круга).

    Пример (ровно как в примере пользователя): n_zones=360, min_zone=90,
    min_coef=0.5, ramp_zones=20 ->
      зона 70:  a=1.0   (начало линейного участка)
      зона 90:  a=0.5   (сам провал)
      зона 110: a=1.0   (конец линейного участка)
      везде за пределами [70, 110] - a=1.0
    """
    if ramp_zones < 1:
        raise ValueError("ramp_zones должен быть >= 1")

    # Циклическое расстояние между zone и min_zone: обычная разница по
    # модулю n_zones, а затем берём меньшее из "по часовой"/"против
    # часовой" - иначе зона 359 и зона 2 (для n_zones=360) считались бы
    # далёкими друг от друга (разница 357), хотя физически они соседи
    # (разница по кругу - всего 3).
    raw_diff = abs(zone - min_zone) % n_zones
    dist = min(raw_diff, n_zones - raw_diff)

    if dist >= ramp_zones:
        return 1.0

    frac = dist / ramp_zones
    return min_coef + (1.0 - min_coef) * frac


def prompt_dip_coefficient_params(n_zones: int):
    """Спрашивает у пользователя: в какой зоне минимум, чему он равен, и
    сколько зон в каждую сторону занимает линейный переход обратно к 1.0."""
    while True:
        raw = input(f"Номер зоны с минимальным коэффициентом (1-{n_zones})? ").strip()
        try:
            min_zone = int(raw)
        except ValueError:
            print(f"  Введите целое число от 1 до {n_zones}.")
            continue
        if not (1 <= min_zone <= n_zones):
            print(f"  Введите целое число от 1 до {n_zones}.")
            continue
        break

    while True:
        raw = input("Минимальный коэффициент a (в этой зоне)? ").strip()
        try:
            min_coef = float(raw)
        except ValueError:
            print("  Введите число (например, 0.5).")
            continue
        break

    while True:
        raw = input("Диапазон линейного перехода (сколько зон в КАЖДУЮ "
                    "сторону от минимума до возврата к 1.0)? ").strip()
        try:
            ramp_zones = int(raw)
        except ValueError:
            print("  Введите целое число >= 1.")
            continue
        if ramp_zones < 1:
            print("  Введите целое число >= 1.")
            continue
        break

    return min_zone, min_coef, ramp_zones


# 1. Создать объект. На этом шаге ничего не подключается - только
#    сохраняются настройки (сколько зон, какой BLE-адрес).
tracker = PoseTracker(n_nutation_zones=6, n_precession_zones=N_PRECESSION_ZONES)

# Параметры коэффициента a спрашиваем сразу же, до подключения к BLE - как
# и с числом зон в cli_demo.py, это "настройка сеанса", не зависящая от
# датчиков.
MIN_ZONE, MIN_COEF, RAMP_ZONES = prompt_dip_coefficient_params(N_PRECESSION_ZONES)

# 2. Подключиться. Блокирует выполнение, пока не установится BLE-связь
#    (по умолчанию - ждёт бесконечно, см. timeout= у connect(), если
#    нужно вместо этого поднять исключение через N секунд).
print("Подключаюсь...")
tracker.connect()
print("Подключено.")

# 3. Калибровка - три позы подряд, каждый calibrate_*() блокирует
#    выполнение на несколько секунд (данные усредняются "внутри" вызова).
#    Здесь просто input() без обратного отсчёта - хватает для примера.
input("Встаньте прямо, руки вдоль тела, потом Enter (и замрите на 3 сек)...")
tracker.calibrate_g()

input("Вытяните руку вперёд, потом Enter (и замрите на 1.5 сек)...")
tracker.calibrate_f()

input("Наклонитесь вперёд, потом Enter (и задержитесь на 2 сек)...")
tracker.calibrate_l()

tracker.finalize_calibration()
print("Калибровка готова.\n")

# 4. Собственно "чёрный ящик" в работе: спрашиваем текущее положение руки
#    сколько угодно раз, не зная и не заботясь о том, как эти данные
#    добываются внутри (BLE-пакеты, усреднение кватернионов и т.д.).
try:
    while True:
        result = tracker.get_pose()
        a = linear_dip_coefficient(result.precession_zone, N_PRECESSION_ZONES,
                                    MIN_ZONE, MIN_COEF, RAMP_ZONES)
        print(f"нутация зона {result.nutation_zone}/{result.n_nutation_zones}, "
              f"прецессия зона {result.precession_zone}/{result.n_precession_zones}, "
              f"a={a:.3f}")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    tracker.disconnect()
