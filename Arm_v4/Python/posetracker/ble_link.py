# -*- coding: utf-8 -*-
"""
BLE-слой: подключение к STM32/BNO08x-модулю, приём notify-пакетов,
разбор строки "w1 x1 y1 z1 w2 x2 y2 z2" в два кватерниона, фоновый поток
переподключения.

ПОЧЕМУ ЭТО КЛАСС, А НЕ МОДУЛЬНЫЕ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (как было раньше:
current_chest_quat = np.array(...) прямо в теле модуля):
Глобальные переменные модуля - это ОДИН общий на всю программу экземпляр
состояния. Для одноразового скрипта, который открывает ровно одно
BLE-соединение и живёт, пока не закроют терминал, это работает и не
мешает. Но для переиспользуемой библиотеки это плохо сразу по двум
причинам:
  1. Нельзя создать два независимых соединения в одной программе (например,
     подключиться сразу к двум разным платам) - глобальная переменная одна
     на весь процесс.
  2. Состояние скрыто "где-то в модуле" - непонятно из сигнатур функций,
     что вообще меняется, тяжело писать тесты (тест A может незаметно
     повлиять на тест B через общую глобальную переменную).
Класс решает обе проблемы: состояние (текущие кватернионы, флаг
подключения, буфер приёма) живёт в conkретном объекте (self.), можно
создать сколько угодно независимых BleQuatLink(...) с разными адресами.
"""

import asyncio
import threading
import time
from typing import Callable

import numpy as np
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

from .quat import Quat, average_quaternions

# Значения по умолчанию - тот же модуль/характеристика, что и раньше.
# Их можно переопределить при создании BleQuatLink(...) - см. ниже.
DEFAULT_DEVICE_ADDRESS = "d6:7d:55:ae:d7:54"          # MAC-адрес E104-BT5032A
DEFAULT_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"  # Notify-канал


def parse_ble_line(line: str) -> tuple[Quat, Quat] | None:
    """"w1 x1 y1 z1 w2 x2 y2 z2" -> (chest_quat, arm_quat) или None, если
    строка не распарсилась (мусор/обрывок пакета/отладочная строка "DBG ...").

    ПОМЕНЯНО МЕСТАМИ: физически CS1 на прошивке оказался на руке, CS2 -
    на груди - отсюда arm=parts[0:4], chest=parts[4:8].

    ПОЧЕМУ ЭТО ОТДЕЛЬНАЯ (СВОБОДНАЯ) ФУНКЦИЯ, А НЕ МЕТОД КЛАССА:
    Она не трогает self - не читает и не меняет никакое состояние объекта,
    только str -> tuple. Такие функции полезно оставлять свободными
    (module-level): их проще тестировать по отдельности
    (parse_ble_line("1 0 0 0 1 0 0 0") -> сразу видно ожидаемый результат,
    не нужно поднимать целый BleQuatLink), и явно видно, что функция не
    имеет побочных эффектов - в отличие от методов класса ниже, у которых
    в имени/контексте понятно, что они трогают self.
    """
    try:
        parts = [float(p) for p in line.strip().split(" ") if p]
    except ValueError:
        return None
    if len(parts) != 8:
        return None
    arm_q = np.array(parts[0:4])
    chest_q = np.array(parts[4:8])
    return chest_q, arm_q


class BleQuatLink:
    """Держит BLE-соединение с платой и отдаёт последние принятые
    кватернионы груди/руки. Вся сетевая работа идёт в фоновом потоке
    (bleak - асинхронная библиотека, а конструктору класса удобнее
    остаться синхронным, чтобы не заставлять КАЖДОГО потребителя API
    писать async/await ради простого чтения текущего значения)."""

    def __init__(self, device_address: str = DEFAULT_DEVICE_ADDRESS,
                 char_uuid: str = DEFAULT_CHAR_UUID) -> None:
        self.device_address = device_address
        self.char_uuid = char_uuid

        # ПОЧЕМУ Lock: фоновый поток (BLE) пишет current_chest_quat/
        # current_arm_quat в произвольный момент времени, а поток
        # потребителя (тот, что вызывает get_quats()) читает их тоже в
        # произвольный момент. Без блокировки можно словить редкую, тяжело
        # воспроизводимую ошибку - например, прочитать кватернион в
        # момент, когда он наполовину переписан новым значением. Lock
        # гарантирует, что чтение и запись не пересекаются по времени.
        self._lock = threading.Lock()
        self._current_chest_quat: Quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._current_arm_quat: Quat = np.array([1.0, 0.0, 0.0, 0.0])

        self._connected = False
        self._running = False
        self._rx_buffer = ""
        # threading.Thread | None: до вызова start() потока ещё нет
        # вообще, отсюда None как начальное значение.
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        """True, пока соединение активно. Свойство (property), а не
        обычный атрибут напрямую - в будущем сюда можно добавить логику
        (например, проверку "давно ли было последнее сообщение") не
        ломая внешний интерфейс: снаружи как было self.link.connected,
        так и останется, при этом чтение атрибута выглядит как обычное
        поле, а не вызов функции connected() - удобнее для читателя."""
        return self._connected

    def start(self) -> None:
        """Запускает фоновый поток с BLE-соединением. Неблокирующий
        вызов - возвращается сразу, подключение происходит асинхронно."""
        if self._thread is not None:
            return  # уже запущено - повторный старт ничего не делает
        self._running = True
        self._thread = threading.Thread(target=self._thread_entry, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Останавливает фоновый поток (используется при выходе из
        программы, чтобы не держать BLE-соединение открытым зря)."""
        self._running = False

    def wait_connected(self, timeout: float | None = None, poll_interval: float = 0.2,
                        on_waiting: Callable[[], None] | None = None) -> None:
        """Блокирует вызывающий поток, пока не появится соединение.

        timeout=None (по умолчанию) - ждать бесконечно, как и в исходном
        скрипте (там просто печаталось предупреждение каждые 15 секунд и
        ожидание продолжалось). timeout=число секунд - бросить
        TimeoutError, если за это время не подключились (полезно для
        автоматических сценариев, где вечное ожидание недопустимо).

        on_waiting - необязательный колбэк без аргументов, вызывается
        каждые ~15 секунд, пока подключения нет. Через него consumer сам
        решает, ПЕЧАТАТЬ что-то или нет - библиотека сама print() не
        делает (см. общее объяснение в tracker.py про то, почему I/O с
        консолью - забота потребителя, а не библиотеки)."""
        t0 = time.time()
        last_notify = t0
        while not self._connected:
            now = time.time()
            if timeout is not None and now - t0 > timeout:
                raise TimeoutError(
                    f"Не удалось подключиться к {self.device_address} за {timeout} сек")
            if on_waiting is not None and now - last_notify > 15.0:
                last_notify = now
                on_waiting()
            time.sleep(poll_interval)

    def get_quats(self) -> tuple[Quat, Quat]:
        """Возвращает КОПИИ последних принятых кватернионов (chest, arm).
        Копия (.copy()), а не сами массивы - чтобы вызывающий код не мог
        случайно изменить внутреннее состояние объекта, потрогав
        возвращённый numpy-массив на месте."""
        with self._lock:
            return self._current_chest_quat.copy(), self._current_arm_quat.copy()

    def sample_window(self, duration: float, avg_start_frac: float) -> tuple[Quat | None, Quat | None]:
        """Забирает текущие кватернионы в течение duration секунд,
        усредняя только вторую часть окна (после duration*avg_start_frac)
        - чтобы в среднее попадало удержание позы, а не переходный
        процесс перехода в неё. Возвращает (chest_avg, arm_avg) - каждый
        либо усреднённый кватернион, либо None, если за всё окно не
        пришло ни одного сэмпла (например, BLE не был подключён)."""
        t0 = time.time()
        quats_c, quats_a = [], []
        while time.time() - t0 < duration:
            q_c, q_a = self.get_quats()
            if time.time() - t0 > duration * avg_start_frac:
                quats_c.append(q_c)
                quats_a.append(q_a)
            time.sleep(0.02)
        return average_quaternions(quats_c), average_quaternions(quats_a)

    # ------------------------- внутренняя механика -------------------------
    # Всё, что ниже (методы с ведущим "_"), не входит в публичный API
    # пакета - это соглашение Python (в отличие от C, где static в файле
    # физически запрещает доступ извне, здесь это просто сигнал "не
    # обращайтесь к этому напрямую, это может измениться без
    # предупреждения"). Публичный API - это ровно то, что переэкспортирует
    # posetracker/__init__.py.

    def _on_notify(self, _sender: BleakGATTCharacteristic, data: bytearray) -> None:
        """Колбэк bleak: вызывается на каждый пришедший BLE-пакет.
        Сигнатура (_sender, data) продиктована самой bleak - именно так
        она вызывает переданный в start_notify колбэк, менять нельзя.
        _sender с подчёркиванием - параметр объявлен, но не используется
        внутри тела метода (нам не важно, какая именно характеристика
        прислала данные - у нас подписка только на одну)."""
        try:
            chunk = data.decode("utf-8")
        except UnicodeDecodeError:
            return
        self._rx_buffer += chunk
        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            parsed = parse_ble_line(line)
            if parsed is not None:
                chest_q, arm_q = parsed
                with self._lock:
                    self._current_chest_quat = chest_q
                    self._current_arm_quat = arm_q

    async def _client_loop(self) -> None:
        """Асинхронный цикл подключения/переподключения. bleak - это
        asyncio-библиотека, поэтому сама логика соединения обязана быть
        async, а вот наружу (в остальной пакет и в потребителя) эта
        асинхронность не просачивается - см. _thread_entry ниже."""
        while self._running:
            try:
                async with BleakClient(self.device_address) as client:
                    await client.start_notify(self.char_uuid, self._on_notify)
                    self._connected = True
                    while self._running and client.is_connected:
                        await asyncio.sleep(0.2)
                    self._connected = False
            except Exception:
                self._connected = False
                await asyncio.sleep(3.0)

    def _thread_entry(self) -> None:
        """Точка входа фонового потока: заводит собственный asyncio event
        loop (asyncio.run) внутри ОБЫЧНОГО потока threading.Thread.

        ПОЧЕМУ ПОТОК + ASYNCIO ВМЕСТЕ: bleak требует asyncio event loop,
        а вызывающему коду (calibrate(), get_pose() и т.д. в tracker.py)
        удобнее оставаться обычным синхронным кодом, без async/await на
        каждом шагу - иначе ЛЮБОЙ потребитель этой библиотеки был бы
        обязан сам разбираться в asyncio, даже если ему нужно только
        "дай мне текущий кватернион". Поток-обёртка прячет asyncio внутри
        библиотеки, наружу отдаётся обычный синхронный API."""
        asyncio.run(self._client_loop())
