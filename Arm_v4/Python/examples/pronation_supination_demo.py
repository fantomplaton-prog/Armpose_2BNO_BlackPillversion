# -*- coding: utf-8 -*-
"""
Консольная демонстрация пакета pronation_supination: подключение, три
калибровочные позы, непрерывный вывод угла. Сравни с examples/cli_demo.py
(тот же паттерн: вся консольная "обвязка" - input()/countdown()/print() -
здесь, а не в самом пакете, который ничего не знает про консоль).

ЗАПУСК:
Пакет должен быть "виден" интерпретатору - как и с posetracker, проще
всего один раз поставить в editable-режиме (из папки Python/):
    pip install -e .
и дальше просто:
    python examples/pronation_supination_demo.py
"""

import math
import time

from pronation_supination import PronSupTracker

DURATION_HANG = 3.0     # рука прямая, висит вдоль тела
DURATION_FORWARD = 1.5  # рука прямая, вытянута вперёд горизонтально
DURATION_ZERO = 2.0     # локоть согнут ~90°, ладонь строго вверх

PRINT_RATE_HZ = 10.0


def countdown(seconds):
    n = int(math.ceil(seconds))
    for i in range(n, 0, -1):
        print(f"  {i}...", end=" ", flush=True)
        time.sleep(min(1.0, seconds - (n - i)))
    print()


def run_calibration(tracker: PronSupTracker):
    input("Поза HANG: рука прямая (локоть разогнут), висит вдоль тела. "
          "Нажмите Enter и замрите...")
    countdown(DURATION_HANG)
    tracker.calibrate_hang(DURATION_HANG)
    print("  поза HANG снята.\n")

    input("Поза FORWARD: рука прямая (локоть по-прежнему разогнут), вытянута "
          "вперёд горизонтально. Нажмите Enter и замрите...")
    countdown(DURATION_FORWARD)
    tracker.calibrate_forward(DURATION_FORWARD)
    print("  поза FORWARD снята.\n")

    input("Поза ZERO: согните локоть примерно на 90°, ладонь строго вверх. "
          "Нажмите Enter и замрите...")
    countdown(DURATION_ZERO)
    tracker.calibrate_zero(DURATION_ZERO)
    print("  поза ZERO снята.\n")

    tracker.finalize_calibration()
    print("Калибровка готова.\n")


def main():
    tracker = PronSupTracker()

    print("Подключаюсь...")

    def on_waiting():
        print("Не удалось подключиться за 15 сек. Проверьте, что модуль "
              "включён и STM32 передаёт данные. Продолжаю ждать в фоне...")

    tracker.connect(on_waiting=on_waiting)
    print("Подключено.\n")

    run_calibration(tracker)
    print("Начинаю вывод угла (Ctrl+C для остановки)...\n")

    print_period = 1.0 / PRINT_RATE_HZ
    last_print = 0.0
    try:
        while True:
            now = time.time()
            if now - last_print >= print_period:
                last_print = now
                angle = tracker.get_angle()
                link_str = "" if tracker.connected else "  [BLE ОТКЛЮЧЕНО]"
                print(f"Пронация/супинация: {angle:7.1f}°{link_str}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
    finally:
        tracker.disconnect()


if __name__ == "__main__":
    main()
