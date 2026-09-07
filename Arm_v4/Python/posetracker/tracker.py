# -*- coding: utf-8 -*-
"""
PoseTracker - публичный "чёрный ящик" пакета: то единственное, с чем
должен работать потребитель, не заглядывая внутрь ble_link.py/
calibration.py/pose.py/quat.py.

ПОЧЕМУ ЗДЕСЬ НЕТ НИ ОДНОГО print()/input():
Библиотека, которую импортируют, не должна сама решать, ЧТО и КАК
показывать пользователю - это дело конкретной программы-потребителя
(консольный скрипт, GUI, веб-страница...). Если бы PoseTracker сам делал
input("Встаньте прямо...") - то, во-первых, эта библиотека была бы
непригодна для любой не-консольной программы (в GUI-приложении не
почитаешь stdin), а во-вторых, тесты пришлось бы гонять интерактивно
руками. Поэтому здесь только методы вида calibrate_g(duration=...) -
подожди duration секунд, посчитай, сохрани результат - а КОГДА вызвать
этот метод, что написать пользователю перед этим и как показать
обратный отсчёт - решает уже вызывающий код (см. examples/cli_demo.py:
он умеет ровно то же самое, что старый Get_calculate_show.py, но вся
"обвязка" с input()/print() лежит там, а не в библиотеке).
"""

from typing import Callable

from .ble_link import DEFAULT_CHAR_UUID, DEFAULT_DEVICE_ADDRESS, BleQuatLink
from .calibration import DEFAULT_SIGN_LEAN_AXIS, Calibration, compute_calibration
from .pose import PoseResult, classify_pose_grid, compute_outputs
from .quat import Quat

# Длительности калибровочных окон по умолчанию, сек. - вынесены сюда как
# именованные константы (а не "магические числа" 3.0/1.5/2.0россыпью по
# коду), чтобы одно и то же значение не пришлось синхронизировать в
# нескольких местах, если его когда-нибудь захочется поменять.
DEFAULT_DURATION_G = 3.0   # стоя прямо, руки вдоль тела
DEFAULT_DURATION_F = 1.5   # рука вытянута вперёд
DEFAULT_DURATION_L = 2.0   # наклон вперёд (держать позу к концу окна)


class PoseTracker:
    """Высокоуровневый API: подключиться -> откалиброваться -> спрашивать
    текущее положение руки (по сетке зон нутация x прецессия).

    Типичный сценарий использования (см. также examples/cli_demo.py):

        tracker = PoseTracker(n_nutation_zones=6, n_precession_zones=8)
        tracker.connect()                       # ждёт BLE-подключение
        tracker.calibrate_g(3.0)                 # поза G - удерживать 3 сек
        tracker.calibrate_f(1.5)                 # поза F - удерживать 1.5 сек
        tracker.calibrate_l(2.0)                 # поза L - удерживать 2 сек
        tracker.finalize_calibration()           # посчитать Calibration
        while True:
            result = tracker.get_pose()          # PoseResult с текущими данными
            print(result.nutation_zone, result.precession_zone)
    """

    def __init__(self,
                 n_nutation_zones: int,
                 n_precession_zones: int,
                 device_address: str = DEFAULT_DEVICE_ADDRESS,
                 char_uuid: str = DEFAULT_CHAR_UUID,
                 sign_lean_axis: float = DEFAULT_SIGN_LEAN_AXIS) -> None:
        if n_nutation_zones < 1 or n_precession_zones < 1:
            raise ValueError("n_nutation_zones и n_precession_zones должны быть >= 1")

        self.n_nutation_zones = n_nutation_zones
        self.n_precession_zones = n_precession_zones
        self.sign_lean_axis = sign_lean_axis
        # Публично, а не только внутри self._link - потребителю (например,
        # для строки "Подключение к ...") незачем знать, что адрес вообще
        # хранится в отдельном внутреннем объекте BleQuatLink.
        self.device_address = device_address

        # Композиция, а не наследование: PoseTracker ИМЕЕТ BleQuatLink
        # (хранит его в self._link), а не ЯВЛЯЕТСЯ BleQuatLink (не
        # class PoseTracker(BleQuatLink)). Наследование здесь было бы не
        # к месту - PoseTracker не "это разновидность BLE-соединения",
        # он ИСПОЛЬЗУЕТ BLE-соединение как один из своих внутренних
        # инструментов, наравне с калибровкой. Общее практическое правило:
        # наследование - для "Y это разновидность X", композиция - для
        # "Y пользуется X" (в большинстве практических случаев это и есть
        # правильный выбор, наследование нужно реже, чем кажется).
        self._link = BleQuatLink(device_address, char_uuid)

        # Сюда лягут кватернионы калибровочных поз по мере их сбора, а
        # потом - готовая Calibration. Пока калибровка не завершена,
        # self._cal остаётся None, и get_pose() явно откажет с понятной
        # ошибкой, а не тихо вернёт мусор.
        self._q_c_g: Quat | None = None
        self._q_a_g: Quat | None = None
        self._q_c_f: Quat | None = None
        self._q_a_f: Quat | None = None
        self._q_c_l: Quat | None = None
        self._cal: Calibration | None = None

    # ------------------------------ подключение ------------------------------

    @property
    def connected(self) -> bool:
        return self._link.connected

    def connect(self, timeout: float | None = None,
                on_waiting: Callable[[], None] | None = None) -> None:
        """Запускает BLE-соединение и ждёт, пока оно установится.
        timeout=None - ждать бесконечно (как в исходном скрипте)."""
        self._link.start()
        self._link.wait_connected(timeout=timeout, on_waiting=on_waiting)

    def disconnect(self) -> None:
        self._link.stop()

    # ------------------------------ калибровка ------------------------------
    # Три метода calibrate_g/f/l ниже - каждый блокирует вызывающий поток
    # на duration секунд и просто собирает данные. Они специально НЕ
    # объединены в один calibrate() "на всё сразу" - потребителю нужно
    # вставить между ними свой input()/print()/обратный отсчёт (см.
    # docstring класса выше), и раздельные методы дают ему точки, куда это
    # вставить.

    def calibrate_g(self, duration: float = DEFAULT_DURATION_G) -> None:
        """Поза G: стоя прямо, руки свободно вдоль тела."""
        q_c, q_a = self._link.sample_window(duration, avg_start_frac=0.4)
        if q_c is None or q_a is None:
            raise RuntimeError("Калибровка G: не удалось получить данные с BLE")
        self._q_c_g, self._q_a_g = q_c, q_a

    def calibrate_f(self, duration: float = DEFAULT_DURATION_F) -> None:
        """Поза F: рука вытянута строго вперёд, корпус не двигается."""
        q_c, q_a = self._link.sample_window(duration, avg_start_frac=0.4)
        if q_c is None or q_a is None:
            raise RuntimeError("Калибровка F: не удалось получить данные с BLE")
        self._q_c_f, self._q_a_f = q_c, q_a

    def calibrate_l(self, duration: float = DEFAULT_DURATION_L) -> None:
        """Поза L: наклон корпуса вперёд из положения стоя."""
        q_c, _ = self._link.sample_window(duration, avg_start_frac=0.5)
        if q_c is None:
            raise RuntimeError("Калибровка L: не удалось получить данные с BLE")
        self._q_c_l = q_c

    def finalize_calibration(self) -> Calibration:
        """Считает Calibration по уже собранным G/F/L (методы выше должны
        быть вызваны заранее хотя бы один раз). Результат сохраняется
        внутри трекера (используется в get_pose()) и одновременно
        возвращается вызывающему коду - например, чтобы вывести
        cal.delta_psi_deg на экран, как это делал исходный скрипт."""
        missing = [name for name, val in (
            ("G", self._q_c_g), ("F", self._q_c_f), ("L", self._q_c_l),
        ) if val is None]
        if missing:
            raise RuntimeError(
                f"Нельзя завершить калибровку - не выполнены позы: {', '.join(missing)}")

        self._cal = compute_calibration(
            self._q_c_g, self._q_a_g, self._q_c_f, self._q_a_f, self._q_c_l,
            sign_lean_axis=self.sign_lean_axis,
        )
        return self._cal

    def recalibrate_heading(self, f_duration: float = DEFAULT_DURATION_F,
                             l_duration: float = DEFAULT_DURATION_L) -> Calibration:
        """Переделывает только F и L (те калибровки, что зависят от курса
        и потому могут разойтись со временем - см. объяснение в докстринге
        исходного скрипта про дрейф курса без магнитометра), используя уже
        сохранённые G с самой первой калибровки. Удобный шорткат вместо
        отдельных calibrate_f()+calibrate_l()+finalize_calibration()."""
        if self._q_c_g is None:
            raise RuntimeError("Нельзя перекалиброваться - ещё не было ни одной calibrate_g()")
        self.calibrate_f(f_duration)
        self.calibrate_l(l_duration)
        return self.finalize_calibration()

    # -------------------------------- вывод -----------------------------------

    def get_pose(self) -> PoseResult:
        """Текущее положение руки: углы + номера зон по заданной сетке.
        Требует завершённой калибровки (finalize_calibration())."""
        if self._cal is None:
            raise RuntimeError("Трекер ещё не откалиброван - вызовите calibrate_g/f/l "
                                "и finalize_calibration() перед get_pose()")

        q_chest, q_arm = self._link.get_quats()
        nutation, precession, tilt_xz, tilt_yz = compute_outputs(q_chest, q_arm, self._cal)
        nut_zone, prec_zone = classify_pose_grid(
            nutation, precession, self.n_nutation_zones, self.n_precession_zones)

        return PoseResult(
            nutation_deg=nutation,
            precession_deg=precession,
            tilt_xz_deg=tilt_xz,
            tilt_yz_deg=tilt_yz,
            nutation_zone=nut_zone,
            precession_zone=prec_zone,
            n_nutation_zones=self.n_nutation_zones,
            n_precession_zones=self.n_precession_zones,
        )
