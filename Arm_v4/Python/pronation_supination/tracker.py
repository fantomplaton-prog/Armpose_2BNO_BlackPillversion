# -*- coding: utf-8 -*-
"""
PronSupTracker - публичный "чёрный ящик" пакета: то единственное, с чем
должен работать потребитель, не заглядывая внутрь calibration.py/pose.py и
уж тем более в BLE-слой posetracker.

ПОЧЕМУ ПЕРЕИСПОЛЬЗУЕТСЯ BleQuatLink ИЗ posetracker, А НЕ СВОЙ BLE-СЛОЙ:
Протокол передачи (формат BLE-строки "w1 x1 y1 z1 w2 x2 y2 z2", MAC-адрес,
UUID характеристики) - тот же самый, прошивка STM32 не менялась. BleQuatLink
не знает и не должен знать о физическом смысле точек крепления датчиков -
он просто отдаёт "первый"/"второй" кватернион. Разные пакеты (posetracker
для нутации/прецессии, pronation_supination для скрутки предплечья) могут
интерпретировать эту же пару кватернионов по-разному - см. маппинг ролей
ниже. Переиспользование вместо копирования BLE-кода - тот же принцип, что
и с posetracker.quat (не плодить вторую копию уже проверенного кода).

ВАЖНО ПРО ФИЗИЧЕСКОЕ ПОДКЛЮЧЕНИЕ (см. разговор с пользователем):
Датчик, который физически был на руке (в posetracker.ble_link исторически
называется "arm", это первый кватернион в BLE-строке), остался на
ПРЕДПЛЕЧЬЕ без изменений. Датчик, который физически был на груди
(исторически "chest", второй кватернион), теперь переставлен на ПЛЕЧО.
Поэтому маппинг ролей здесь:

    chest_q (из BleQuatLink) -> физически на плече      -> q_upper
    arm_q   (из BleQuatLink) -> физически на предплечье  -> q_forearm

Если датчики физически поменяют местами ещё раз - поправить нужно только
метод _sample() ниже, остального это не касается.
"""

from typing import Callable

from posetracker.ble_link import DEFAULT_CHAR_UUID, DEFAULT_DEVICE_ADDRESS, BleQuatLink
from posetracker.quat import Quat

from .calibration import PronSupCalibration, calibrate_pronation_supination
from .pose import DEFAULT_SIGN_TWIST, compute_pronation_supination

# Длительности калибровочных окон по умолчанию, сек. - как DEFAULT_DURATION_*
# в posetracker/tracker.py, вынесены именованными константами, чтобы не
# разбредались "магическими числами" по коду.
DEFAULT_DURATION_HANG = 3.0     # рука прямая, висит вдоль тела
DEFAULT_DURATION_FORWARD = 1.5  # рука прямая, вытянута вперёд горизонтально
DEFAULT_DURATION_ZERO = 2.0     # локоть согнут ~90°, ладонь строго вверх


class PronSupTracker:
    """Высокоуровневый API: подключиться -> откалиброваться (3 позы) ->
    спрашивать текущий угол пронации/супинации.

    Типичный сценарий использования (см. также examples/pronation_supination_demo.py):

        tracker = PronSupTracker()
        tracker.connect()                # ждёт BLE-подключение
        tracker.calibrate_hang()          # поза hang - удерживать 3 сек
        tracker.calibrate_forward()       # поза forward - удерживать 1.5 сек
        tracker.calibrate_zero()          # поза zero - удерживать 2 сек
        tracker.finalize_calibration()    # посчитать PronSupCalibration
        while True:
            angle = tracker.get_angle()   # градусы, 0 = поза zero
            print(angle)
    """

    def __init__(self,
                 device_address: str = DEFAULT_DEVICE_ADDRESS,
                 char_uuid: str = DEFAULT_CHAR_UUID,
                 sign_twist: float = DEFAULT_SIGN_TWIST) -> None:
        self.sign_twist = sign_twist
        self.device_address = device_address

        # Композиция, а не копирование - см. docstring модуля выше и тот же
        # аргумент про PoseTracker/BleQuatLink в posetracker/tracker.py.
        self._link = BleQuatLink(device_address, char_uuid)

        self._q_upper_hang: Quat | None = None
        self._q_forearm_hang: Quat | None = None
        self._q_upper_forward: Quat | None = None
        self._q_forearm_forward: Quat | None = None
        self._q_upper_zero: Quat | None = None
        self._q_forearm_zero: Quat | None = None
        self._cal: PronSupCalibration | None = None

    # ------------------------------ подключение ------------------------------

    @property
    def connected(self) -> bool:
        """True, пока BLE-соединение активно (см. тот же паттерн в
        posetracker.ble_link.BleQuatLink.connected - почему это свойство,
        а не обычный метод/атрибут)."""
        return self._link.connected

    def connect(self, timeout: float | None = None,
                on_waiting: Callable[[], None] | None = None) -> None:
        """Запускает BLE-соединение и ждёт, пока оно установится.
        timeout=None - ждать бесконечно."""
        self._link.start()
        self._link.wait_connected(timeout=timeout, on_waiting=on_waiting)

    def disconnect(self) -> None:
        """Останавливает фоновый BLE-поток (используется при выходе из
        программы, чтобы не держать соединение открытым зря)."""
        self._link.stop()

    # ------------------------------ калибровка ------------------------------
    # Три метода calibrate_hang/forward/zero - каждый блокирует вызывающий
    # поток на duration секунд и просто собирает данные, как и в PoseTracker.
    # Специально НЕ объединены в один calibrate() - потребителю нужно
    # вставить между ними свой input()/print()/обратный отсчёт.

    def _sample(self, duration: float, avg_start_frac: float) -> tuple[Quat, Quat]:
        """Возвращает (q_upper, q_forearm) - маппинг ролей см. docstring
        модуля выше."""
        chest_q, arm_q = self._link.sample_window(duration, avg_start_frac=avg_start_frac)
        if chest_q is None or arm_q is None:
            raise RuntimeError("Не удалось получить данные с BLE за время калибровки")
        return chest_q, arm_q  # (q_upper, q_forearm)

    def calibrate_hang(self, duration: float = DEFAULT_DURATION_HANG) -> None:
        """Поза hang: рука прямая (локоть разогнут), висит вдоль тела."""
        self._q_upper_hang, self._q_forearm_hang = self._sample(duration, avg_start_frac=0.4)

    def calibrate_forward(self, duration: float = DEFAULT_DURATION_FORWARD) -> None:
        """Поза forward: рука прямая (локоть по-прежнему разогнут), вытянута
        вперёд горизонтально."""
        self._q_upper_forward, self._q_forearm_forward = self._sample(duration, avg_start_frac=0.4)

    def calibrate_zero(self, duration: float = DEFAULT_DURATION_ZERO) -> None:
        """Поза zero: локоть согнут ~90°, ладонь строго вверх (анатомический
        ноль пронации/супинации)."""
        self._q_upper_zero, self._q_forearm_zero = self._sample(duration, avg_start_frac=0.5)

    def finalize_calibration(self) -> PronSupCalibration:
        """Считает PronSupCalibration по уже собранным hang/forward/zero
        (методы выше должны быть вызваны заранее хотя бы по разу). Результат
        сохраняется внутри трекера (используется в get_angle()) и
        одновременно возвращается вызывающему коду."""
        missing = [name for name, val in (
            ("hang", self._q_upper_hang), ("forward", self._q_upper_forward),
            ("zero", self._q_upper_zero),
        ) if val is None]
        if missing:
            raise RuntimeError(
                f"Нельзя завершить калибровку - не выполнены позы: {', '.join(missing)}")

        self._cal = calibrate_pronation_supination(
            self._q_upper_hang, self._q_forearm_hang,
            self._q_upper_forward, self._q_forearm_forward,
            self._q_upper_zero, self._q_forearm_zero,
        )
        return self._cal

    # -------------------------------- вывод -----------------------------------

    def get_angle(self) -> float:
        """Текущий угол пронации/супинации (градусы). Требует завершённой
        калибровки (finalize_calibration())."""
        if self._cal is None:
            raise RuntimeError("Трекер ещё не откалиброван - вызовите calibrate_hang/forward/zero "
                                "и finalize_calibration() перед get_angle()")

        chest_q, arm_q = self._link.get_quats()
        q_upper, q_forearm = chest_q, arm_q  # маппинг ролей см. docstring модуля
        return compute_pronation_supination(q_upper, q_forearm, self._cal, sign_twist=self.sign_twist)
